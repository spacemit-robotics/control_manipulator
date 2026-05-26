/*
* Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
* SPDX-License-Identifier: Apache-2.0
*/
#ifdef HAVE_MOTOR
/**
* @file drv_uart_so101.c
* @brief SO-101 arm driver using Feetech SMS/STS motors over UART.
*
* Handles 5 DOF arm joints (motor IDs 1-5).
* Gripper (motor ID 6) is managed separately by grasp module.
*
* Motor API note: current Feetech driver uses SI units:
*   - pos_des: rad
*   - vel_des: rad/s
*   - state.pos/state.vel return rad/rad/s
*
* Assemble / Calibrate 流程 (参考 so101_ros2):
*   - assemble: 断电组装 → 配置舵机参数 (PID/加速度/延迟) → 上电
*   - calibrate: 释放扭矩 → 手动移到中位 → 记录 homing_offset → 记录行程范围
*/

#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "kinematics_interface.h"
#include "motor.h"
#include "so101_utils.h"
#include "sts3215_regs.h"

#include "../manipulator_core.h"

/* REG_TORQUE_ENABLE, REG_LOCK, REG_MODE, REG_ACC, REG_OFS_L,
* REG_MIN_ANGLE_L, REG_MAX_ANGLE_L, REG_PRESENT_POS_L
* 均已在 sts3215_regs.h 中定义，无需在此重复。 */

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif


/* 默认运动速度 (rad/s) */
#define SO101_DEFAULT_VEL_RAD_S 1.0f
/* 到位判定阈值 (rad) */
#define SO101_REACHED_EPS_RAD 0.03f

/* 关节名称 (与 so101_ros2 对齐) */
static const char *so101_joint_names[SO101_ARM_MOTOR_COUNT] = {
    "shoulder_pan",   /* ID 1 */
    "shoulder_lift",  /* ID 2 */
    "elbow_flex",     /* ID 3 */
    "wrist_flex",     /* ID 4 */
    "wrist_roll",     /* ID 5 */
};

/* ==========================================================================
* Configuration (passed via manip_alloc args)
* ========================================================================== */

/* ==========================================================================
* Calibration Data
* ========================================================================== */

/* 默认校准文件路径 */
#define SO101_CALIBRATION_PATH SO101_DEFAULT_CALIBRATION_PATH

/* ==========================================================================
* Private Data
* ========================================================================== */

/* 直线插补参数 */
#define SO101_LINE_MAX_STEPS  64    /* 轨迹最大步数 */
#define SO101_LINE_STEP_M     0.005 /* 插补步长 5mm */

/**
* @brief 笛卡尔直线轨迹 (move_line 使用)
*
* 存储预计算的关节空间 waypoint 序列。
* tick() 逐步发送，实现 TCP 直线运动。
*/
struct so101_line_traj {
    struct motor_cmd steps[SO101_LINE_MAX_STEPS][SO101_ARM_MOTOR_COUNT];
    int total;     /* 总步数   (0 = 无轨迹) */
    int current;   /* 当前步索引 */
};

/**
* @brief SO-101 驱动私有数据
*
* 每个 SO-101 实例拥有 5 个 motor_dev 指针，
* 分别对应 5 个关节舵机。
*/
struct so101_priv {
    struct motor_dev *motors[SO101_ARM_MOTOR_COUNT]; /* 电机句柄 */
    struct so101_calibration calib;             /* 校准数据 */
    bool assembled;                            /* 已完成 assemble */

    /* 运动控制状态 - 用于 tick() 持续发送命令 */
    struct motor_cmd target_cmds[SO101_ARM_MOTOR_COUNT]; /* 目标命令 */
    bool has_target;                                /* 有待执行目标 */

    /* 直线轨迹 (move_line) */
    struct so101_line_traj line;
};

/* ==========================================================================
* Register Access Helpers
*
* Thin wrappers around motor_set_paras / motor_get_paras.
* address 参数为 uint8_t 寄存器地址，
* data_len 区分 byte (1) / word (2) 操作。
* ========================================================================== */

static inline int reg_write_byte(struct motor_dev *dev,
                                    uint8_t reg, uint8_t val) {
    return motor_set_paras(dev, &reg, &val, 1);
}

static inline int reg_write_word(struct motor_dev *dev,
                                    uint8_t reg, uint16_t val) {
    return motor_set_paras(dev, &reg, &val, 2);
}

static inline int reg_read_byte(struct motor_dev *dev, uint8_t reg) {
    uint8_t val = 0;
    if (motor_get_paras(dev, &reg, &val, 1) != 0)
        return -1;
    return (int)val;
}

static inline int reg_read_word(struct motor_dev *dev, uint8_t reg) {
    uint16_t val = 0;
    if (motor_get_paras(dev, &reg, &val, 2) != 0)
        return -1;
    return (int)val;
}

/* ==========================================================================
* Torque / EPROM Control Helpers
* ========================================================================== */

/**
* @brief 关闭扭矩
*
* 新版 motor 驱动 (motor_set_paras) 会自动处理
* EPROM 区域的 lock/unlock，无需手动写 REG_LOCK。
*/
static int so101_disable_torque(struct motor_dev *motor) {
    return reg_write_byte(motor, REG_TORQUE_ENABLE, 0);
}

/**
* @brief 使能扭矩
*/
static int so101_enable_torque(struct motor_dev *motor) {
    return reg_write_byte(motor, REG_TORQUE_ENABLE, 1);
}

/**
* @brief 批量关闭所有关节扭矩
*/
static int so101_disable_torque_all(struct motor_dev **motors, int count) {
    for (int i = 0; i < count; i++) {
        if (so101_disable_torque(motors[i]) != 0) {
            fprintf(stderr, "[SO101] disable_torque failed: joint %d (%s)\n",
                            i, so101_joint_names[i]);
            return -1;
        }
    }
    return 0;
}

/**
* @brief 批量使能所有关节扭矩
*/
static int so101_enable_torque_all(struct motor_dev **motors, int count) {
    for (int i = 0; i < count; i++) {
        if (so101_enable_torque(motors[i]) != 0) {
            fprintf(stderr, "[SO101] enable_torque failed: joint %d (%s)\n",
                            i, so101_joint_names[i]);
            return -1;
        }
    }
    return 0;
}

/* ==========================================================================
* Sign-Magnitude Encoding (STS3215 specific)
*
* STS3215 舵机的 Homing_Offset 等寄存器使用 sign-magnitude 编码，
* 而非二进制补码 (two's complement)。
*
* 参考 lerobot/motors/feetech/tables.py:
*   STS_SMS_SERIES_ENCODINGS_TABLE = {
*       "Homing_Offset": 11,   // sign bit at bit 11
*       "Present_Position": 15,
*       ...
*   }
*
* 编码格式:
*   正值:  低 N 位 = 幅值
*   负值:  bit[sign_bit] = 1, 低 N 位 = |值|
*
* 例: sign_bit=11, value=-100
*   → 0x0800 | 100 = 0x0864
* ========================================================================== */

#define STS3215_HOMING_OFFSET_SIGN_BIT  11

/**
* @brief 将有符号整数编码为 sign-magnitude 格式
*
* @param value    有符号值
* @param sign_bit 符号位位置 (从 0 开始)
* @return 编码后的无符号值
*/
static inline uint16_t encode_sign_magnitude(int16_t value, int sign_bit) {
    uint16_t mag = (uint16_t)abs(value);
    if (value < 0)
        mag |= (1u << sign_bit);
    return mag;
}

/**
* @brief 将 sign-magnitude 格式解码为有符号整数
*
* @param raw      原始无符号值
* @param sign_bit 符号位位置 (从 0 开始)
* @return 解码后的有符号值
*/
static inline int16_t decode_sign_magnitude(uint16_t raw, int sign_bit) {
    int16_t mag = (int16_t)(raw & ((1u << sign_bit) - 1));
    if (raw & (1u << sign_bit))
        mag = -mag;
    return mag;
}

/* ==========================================================================
* Motor Configuration (PID / Acceleration / Delay)
* ========================================================================== */

/**
* @brief 配置单个舵机的运行参数
*
* 参考 so101_ros2 的 configure_motors() + set_bus_baudrate() 流程：
*   - Return_Delay_Time = 0 (减少通信延迟)
*   - Maximum_Acceleration = 254 (最大加速度)
*   - Acceleration (SRAM) = 254
*   - Operating_Mode = 0 (位置伺服模式)
*   - P = 16, I = 0, D = 0 (SO-101 推荐 PID)
*
* 注意: EPROM 寄存器写入前需要先 disable_torque() 解锁。
*
* @param motor 电机句柄
* @param name  关节名称 (用于日志)
* @return 0 成功，-1 失败
*/
static int so101_configure_motor(struct motor_dev *motor, const char *name) {
    /* 写 EPROM 前必须先解锁 (Torque=0, Lock=0) */
    if (so101_disable_torque(motor) != 0) {
        fprintf(stderr, "[SO101] disable_torque failed: %s\n", name);
        return -1;
    }

    /* EPROM 区域: Return_Delay_Time, PID, 运行模式 */

    /* Return_Delay_Time = 0 */
    if (reg_write_byte(motor, STS3215_RETURN_DELAY_TIME, 0) != 0) {
        fprintf(stderr, "[SO101] set Return_Delay_Time failed: %s\n", name);
        return -1;
    }

    /* PID: P=16, I=0, D=0 (降低 P 避免过冲) */
    if (reg_write_byte(motor, STS3215_P_COEFFICIENT,
                        STS3215_SO101_P) != 0 ||
        reg_write_byte(motor, STS3215_I_COEFFICIENT,
                        STS3215_SO101_I) != 0 ||
        reg_write_byte(motor, STS3215_D_COEFFICIENT,
                        STS3215_SO101_D) != 0) {
        fprintf(stderr, "[SO101] set PID failed: %s\n", name);
        return -1;
    }

    /* Operating_Mode = 0 (位置伺服模式) */
    if (reg_write_byte(motor, REG_MODE, 0) != 0) {
        fprintf(stderr, "[SO101] set mode failed: %s\n", name);
        return -1;
    }

    /* SRAM 区域: 加速度参数 (不需要解锁) */

    /* Acceleration = 254 (SRAM) */
    if (reg_write_byte(motor, REG_ACC, STS3215_SO101_ACC) != 0) {
        fprintf(stderr, "[SO101] set acc failed: %s\n", name);
        return -1;
    }

    /* 写完 EPROM 后重新锁定 (Torque=1, Lock=1) */
    if (so101_enable_torque(motor) != 0) {
        fprintf(stderr, "[SO101] enable_torque failed: %s\n", name);
        return -1;
    }

    return 0;
}

/**
    * @brief 配置所有 5 个关节舵机
    */
static int so101_configure_motors(struct motor_dev **motors, int count) {
    for (int i = 0; i < count; i++) {
        if (so101_configure_motor(motors[i], so101_joint_names[i]) != 0)
            return -1;
    }
    printf("[SO101] 所有关节参数配置完成 (P=%d I=%d D=%d ACC=%d)\n",
                    STS3215_SO101_P, STS3215_SO101_I, STS3215_SO101_D,
                    STS3215_SO101_ACC);
    return 0;
}

/* ==========================================================================
* Calibration File I/O (simple JSON format)
* ========================================================================== */

/**
* @brief 保存校准数据到文件
*
* 格式与 so101_ros2 兼容的简化 JSON:
* {
*   "joint_0": {"homing_offset": 123, "range_min": 100, "range_max": 3900},
*   ...
* }
*/
static int so101_save_calibration(
        const struct so101_calibration *calib, const char *path) {
    /* 创建目录（如果不存在） */
    char dir_path[256];
    snprintf(dir_path, sizeof(dir_path), "%s", path);
    char *last_slash = strrchr(dir_path, '/');
    if (last_slash) {
        *last_slash = '\0';
        /* 使用 mkdir -p 创建目录 */
        char cmd[512];
        snprintf(cmd, sizeof(cmd), "mkdir -p %s", dir_path);
        system(cmd);
    }

    FILE *f = fopen(path, "w");
    if (!f) {
        fprintf(stderr, "[SO101] 无法写入校准文件: %s (%s)\n",
                        path, strerror(errno));
        return -1;
    }

    fprintf(f, "{\n");
    for (int i = 0; i < SO101_ARM_MOTOR_COUNT; i++) {
        const struct so101_joint_calib *jc = &calib->joints[i];
        fprintf(f,
            "  \"%s\": {\"id\": %d, \"homing_offset\": %d, "
            "\"range_min\": %u, \"range_max\": %u}%s\n",
            so101_joint_names[i], i + 1, jc->homing_offset,
            jc->range_min, jc->range_max,
            (i < SO101_ARM_MOTOR_COUNT - 1) ? "," : "");
    }
    fprintf(f, "}\n");
    fclose(f);

    printf("[SO101] 校准数据已保存到: %s\n", path);
    return 0;
}

/**
* @brief 从文件加载校准数据
*
* 简单的手工 JSON 解析 (避免依赖第三方库)。
* 只需要解析 homing_offset, range_min, range_max 三个字段。
*/
static int so101_load_calibration(struct so101_calibration *calib,
                                    const char *path) {
    FILE *f = fopen(path, "r");
    if (!f) {
        /* 文件不存在不算错误，只是没有校准 */
        return -1;
    }

    char line[256];
    int loaded = 0;

    while (fgets(line, sizeof(line), f) && loaded < SO101_ARM_MOTOR_COUNT) {
        /* 查找 "homing_offset" 关键字来定位数据行 */
        char *p = strstr(line, "homing_offset");
        if (!p)
            continue;

        /* 解析 "id": N */
        int id = 0;
        char *id_p = strstr(line, "\"id\":");
        if (id_p)
            id = atoi(id_p + 5);

        /* 解析 homing_offset, range_min, range_max */
        int hofs = 0;
        unsigned int rmin = 0, rmax = 4095;

        char *hofs_p = strstr(line, "\"homing_offset\":");
        if (hofs_p)
            hofs = atoi(hofs_p + 16);

        char *rmin_p = strstr(line, "\"range_min\":");
        if (rmin_p)
            rmin = (unsigned int)atoi(rmin_p + 12);

        char *rmax_p = strstr(line, "\"range_max\":");
        if (rmax_p)
            rmax = (unsigned int)atoi(rmax_p + 12);

        /* 将 id (1-based) 映射到数组索引 (0-based) */
        int idx = id - 1;
        if (idx >= 0 && idx < SO101_ARM_MOTOR_COUNT) {
            calib->joints[idx].homing_offset = (int16_t)hofs;
            calib->joints[idx].range_min = (uint16_t)rmin;
            calib->joints[idx].range_max = (uint16_t)rmax;
            loaded++;
        }
    }
    fclose(f);

    if (loaded == SO101_ARM_MOTOR_COUNT) {
        calib->valid = true;
        printf("[SO101] 已加载校准数据: %s (%d 个关节)\n", path, loaded);
        return 0;
    }

    fprintf(stderr, "[SO101] 校准文件不完整: 只找到 %d/%d 个关节\n",
                    loaded, SO101_ARM_MOTOR_COUNT);
    return -1;
}

/**
* @brief 将校准数据写入舵机寄存器
*
* 参考 so101_ros2 的 write_calibration():
*   - Homing_Offset (REG_OFS_L, word)
*   - Min_Position_Limit (REG_MIN_ANGLE_L, word)
*   - Max_Position_Limit (REG_MAX_ANGLE_L, word)
*/
static int so101_write_calibration_to_motors(
        struct motor_dev **motors, const struct so101_calibration *calib) {
    if (!calib->valid)
        return -1;

    for (int i = 0; i < SO101_ARM_MOTOR_COUNT; i++) {
        const struct so101_joint_calib *jc = &calib->joints[i];

        /* 写入前先关闭扭矩 (解锁 EPROM) */
        so101_disable_torque(motors[i]);

        /* Homing_Offset — STS3215 使用 sign-magnitude 编码 (sign bit = 11) */
        uint16_t ofs_encoded = encode_sign_magnitude(
                jc->homing_offset, STS3215_HOMING_OFFSET_SIGN_BIT);
        if (reg_write_word(motors[i], REG_OFS_L, ofs_encoded) != 0) {
            fprintf(stderr, "[SO101] write homing_offset failed: %s\n",
                            so101_joint_names[i]);
            return -1;
        }

        /* Min_Position_Limit */
        if (reg_write_word(motors[i], REG_MIN_ANGLE_L,
                            jc->range_min) != 0) {
            fprintf(stderr, "[SO101] write range_min failed: %s\n",
                            so101_joint_names[i]);
            return -1;
        }

        /* Max_Position_Limit */
        if (reg_write_word(motors[i], REG_MAX_ANGLE_L,
                            jc->range_max) != 0) {
            fprintf(stderr, "[SO101] write range_max failed: %s\n",
                            so101_joint_names[i]);
            return -1;
        }

        so101_enable_torque(motors[i]);
    }

    printf("[SO101] 校准数据已写入舵机寄存器\n");
    return 0;
}

/* ==========================================================================
* Assemble Process
* ========================================================================== */

/**
* @brief 组装过程 — 安全地初始化 SO-101 舵机参数
*
* 在首次组装或更换舵机时调用。流程参考 so101_ros2:
*   1. 关闭所有舵机扭矩 (舵机可自由转动)
*   2. 配置舵机参数 (PID / 加速度 / 延迟 / 运行模式)
*   3. 打印提示：用户手动将各关节摆到中位
*   4. (可选) 使能扭矩
*
* ⚠️ 使用场景:
*   - 首次组装机械臂时
*   - 更换新舵机后
*   - 舵机参数被意外修改后
*
* @param motors  5 个关节电机句柄数组
* @param count   关节数量 (5)
* @return 0 成功，-1 失败
*/
int so101_assemble(struct motor_dev **motors, int count) {
    if (!motors || count != SO101_ARM_MOTOR_COUNT)
        return -1;

    printf("[SO101] ===== 开始组装过程 =====\n");
    printf("[SO101] 步骤 1/3: 关闭所有扭矩 (舵机可自由转动)...\n");

    if (so101_disable_torque_all(motors, count) != 0) {
        fprintf(stderr, "[SO101] 关闭扭矩失败\n");
        return -1;
    }
    printf("[SO101]   ✓ 所有舵机扭矩已关闭\n");

    printf("[SO101] 步骤 2/3: 配置舵机参数...\n");
    if (so101_configure_motors(motors, count) != 0) {
        fprintf(stderr, "[SO101] 配置参数失败\n");
        return -1;
    }
    printf("[SO101]   ✓ 参数配置完成\n");

    printf("[SO101] 步骤 3/3: 验证...\n");
    for (int i = 0; i < count; i++) {
        int pos = reg_read_word(motors[i], REG_PRESENT_POS_L);
        int mode = reg_read_byte(motors[i], REG_MODE);
        int p_val = reg_read_byte(motors[i], STS3215_P_COEFFICIENT);
        printf("[SO101]   %s (ID=%d): pos=%d mode=%d P=%d\n",
                        so101_joint_names[i], i + 1, pos, mode, p_val);
    }

    printf("[SO101] ===== 组装完成 =====\n");
    printf("[SO101] 现在可以手动将各关节摆到中位，然后执行校准\n");
    return 0;
}

/* ==========================================================================
* Calibration Process
* ========================================================================== */

/**
* @brief 记录中位偏移 (homing offset)
*
* 参考 so101_ros2 的 set_half_turn_homings():
*   homing_offset = actual_position - 2047
*
* 调用前需确保:
*   - 扭矩已关闭
*   - 用户已将所有关节手动移到中位
*/
static int so101_record_homing_offsets(struct motor_dev **motors,
                                        struct so101_calibration *calib) {
    printf("[SO101] 记录中位偏移...\n");

    /*
    * 重要: 先清除所有舵机的 Homing_Offset，否则 Present_Position
    * 已经被旧偏移修正过，导致新校准叠加旧偏移越来越偏。
    *
    * STS3215 硬件行为: Present_Position = Actual_Position - Homing_Offset
    * 清零后 Present_Position = Actual_Position，确保读到真实物理位置。
    */
    printf("[SO101]   清除旧 Homing_Offset...\n");
    for (int i = 0; i < SO101_ARM_MOTOR_COUNT; i++) {
        /*
        * 只解锁 EPROM 写入偏移清零，不要重新使能扭矩！
        * 校准模式下扭矩必须保持关闭，否则舵机会跳到上次的目标位置。
        */
        so101_disable_torque(motors[i]);
        reg_write_word(motors[i], REG_OFS_L, 0);
        /* 不调用 so101_enable_torque — 保持扭矩关闭 */
    }
    usleep(100000); /* 等待 100ms 让寄存器生效 */

    for (int i = 0; i < SO101_ARM_MOTOR_COUNT; i++) {
        int pos = reg_read_word(motors[i], REG_PRESENT_POS_L);
        if (pos < 0) {
            fprintf(stderr, "[SO101] 读取位置失败: %s\n", so101_joint_names[i]);
            return -1;
        }

        int16_t offset = (int16_t)(pos - STS3215_POSITION_MID);
        calib->joints[i].homing_offset = offset;

        printf("[SO101]   %s: pos=%d → offset=%d\n",
                        so101_joint_names[i], pos, offset);
    }
    return 0;
}

/**
* @brief 记录行程范围
*
* 参考 so101_ros2 的 record_ranges_of_motion():
*   用户手动将每个关节转到最小和最大位置，程序记录。
*
* 简化实现：使用当前位置 ± 合理范围作为默认值，
* 或由用户在交互模式下手动测量。
*
* @param motors  电机句柄数组
* @param calib   校准数据结构体
* @param interactive 是否交互模式 (等待用户移动关节)
* @return 0 成功，-1 失败
*/
static int so101_record_ranges(struct motor_dev **motors,
                                struct so101_calibration *calib,
                                bool interactive) {
    if (!interactive) {
        /* 非交互模式：使用合理的默认行程 */
        for (int i = 0; i < SO101_ARM_MOTOR_COUNT; i++) {
            calib->joints[i].range_min = 100;
            calib->joints[i].range_max = 3995;
            printf("[SO101]   %s: range=[%u, %u] (默认)\n",
                    so101_joint_names[i],
                    calib->joints[i].range_min,
                    calib->joints[i].range_max);
        }
        return 0;
    }

    /* 交互模式：逐个关节记录最小/最大行程 */
    printf("[SO101] 交互式行程记录 — 请手动转动关节到极限位置\n");
    printf("[SO101] (扭矩已关闭，可自由转动)\n\n");

    for (int i = 0; i < SO101_ARM_MOTOR_COUNT; i++) {
        printf("[SO101] ---- %s (ID=%d) ----\n", so101_joint_names[i], i + 1);

        printf("[SO101] 将该关节转到 最小位置，然后按 Enter...");
        while (getchar() != '\n') {}
        int min_pos = reg_read_word(motors[i], REG_PRESENT_POS_L);
        if (min_pos < 0) {
            fprintf(stderr, "[SO101] 读取位置失败\n");
            return -1;
        }

        printf("[SO101] 将该关节转到 最大位置，然后按 Enter...");
        while (getchar() != '\n') {}
        int max_pos = reg_read_word(motors[i], REG_PRESENT_POS_L);
        if (max_pos < 0) {
            fprintf(stderr, "[SO101] 读取位置失败\n");
            return -1;
        }

        /* 确保 min < max */
        if (min_pos > max_pos) {
            int tmp = min_pos;
            min_pos = max_pos;
            max_pos = tmp;
        }

        calib->joints[i].range_min = (uint16_t)min_pos;
        calib->joints[i].range_max = (uint16_t)max_pos;

        printf("[SO101]   %s: range=[%d, %d]\n",
                        so101_joint_names[i], min_pos, max_pos);
    }
    return 0;
}

/**
* @brief 完整校准过程
*
* 参考 so101_ros2 的 calibrate():
*   1. 关闭扭矩 → 用户手动移到中位
*   2. 记录 homing_offset
*   3. 记录行程范围
*   4. 写入舵机寄存器
*   5. 保存到文件
*
* @param motors      电机句柄数组
* @param calib       校准数据 (输出)
* @param interactive 是否交互模式
* @param save_path   校准文件保存路径，NULL 使用默认路径
* @return 0 成功，-1 失败
*/
int so101_calibrate(struct motor_dev **motors,
                                        struct so101_calibration *calib,
                                        bool interactive,
                                        const char *save_path) {
    if (!motors || !calib)
        return -1;

    const char *path = save_path ? save_path : SO101_CALIBRATION_PATH;

    printf("[SO101] ===== 开始校准过程 =====\n");

    /* Step 1: 关闭扭矩 */
    printf("[SO101] 步骤 1/5: 关闭扭矩...\n");
    if (so101_disable_torque_all(motors, SO101_ARM_MOTOR_COUNT) != 0)
        return -1;

    if (interactive) {
            printf("[SO101] 步骤 2/5: 请将所有关节手动移到中间位置，"
                    "然后按 Enter...\n");
        while (getchar() != '\n') {}
    } else {
        printf("[SO101] 步骤 2/5: 读取当前位置作为中位...\n");
        usleep(200000); /* 等待 200ms 让舵机稳定 */
    }

    /* Step 3: 记录 homing offset 并立刻写入舵机 */
    printf("[SO101] 步骤 3/5: 记录中位偏移并写入舵机...\n");
    if (so101_record_homing_offsets(motors, calib) != 0)
        return -1;

    /*
    * 在此立刻将 homing_offset 写入舵机寄存器，确保后续 step 4 读取的
    * Present_Position 已经在 offset-adjusted 坐标系下。
    * 这样行程范围可以直接用于 Min/Max_Position_Limit，无需手动转换。
    *
    * 注意：保持扭矩关闭，用户仍然可以自由转动关节。
    */
    printf("[SO101]   写入 homing_offset 到舵机 (保持扭矩关闭)...\n");
    for (int i = 0; i < SO101_ARM_MOTOR_COUNT; i++) {
        /* EPROM 已在 so101_record_homing_offsets 中解锁 (torque=0) */
        uint16_t ofs_encoded = encode_sign_magnitude(
                calib->joints[i].homing_offset, STS3215_HOMING_OFFSET_SIGN_BIT);
        if (reg_write_word(motors[i], REG_OFS_L, ofs_encoded) != 0) {
            fprintf(stderr, "[SO101] write homing_offset failed: %s\n",
                            so101_joint_names[i]);
            return -1;
        }
        /* 不使能扭矩 — 用户需要继续手动转动关节记录行程 */
    }
    usleep(100000); /* 等待 100ms 让偏移生效 */

    /* 验证：读取当前位置应为 ~2047 (中位已对齐) */
    printf("[SO101]   验证 offset 生效 (期望 Present_Position ≈ 2047):\n");
    for (int i = 0; i < SO101_ARM_MOTOR_COUNT; i++) {
        int pos = reg_read_word(motors[i], REG_PRESENT_POS_L);
        printf("[SO101]     %s: %d\n", so101_joint_names[i], pos);
    }

    /* Step 4: 记录行程范围 (此时 Present_Position 已在
        offset-adjusted 坐标系) */
    printf("[SO101] 步骤 4/5: 记录行程范围 (基于校准后坐标系)...\n");
    if (so101_record_ranges(motors, calib, interactive) != 0)
        return -1;

    calib->valid = true;

    /* Step 5: 写入完整校准数据到舵机并保存 JSON 文件 */
    printf("[SO101] 步骤 5/5: 写入完整校准数据并保存文件...\n");
    if (so101_write_calibration_to_motors(motors, calib) != 0)
        return -1;

    if (so101_save_calibration(calib, path) != 0) {
        fprintf(stderr, "[SO101] 保存文件失败，但寄存器已写入\n");
        /* 非致命: 寄存器已经写入成功 */
    }

    printf("[SO101] ===== 校准完成 =====\n");
    return 0;
}

/* ==========================================================================
* Unit Conversion (rad <-> Feetech ticks)
* ========================================================================== */

/**
* @brief 弧度转 Feetech 刻度
*
* 将弧度值转换为 Feetech 刻度，以中位 (2047) 为基准。
*
* 校准后的坐标系：
*   - 0 rad → 2047 ticks (中位，对应校准时的机械零位)
*   - +π rad → 4095 ticks (顺时针 180°)
*   - -π rad → 0 ticks (逆时针 180°)
*
* Feetech 舵机的 homing_offset 寄存器会自动修正物理位置，
* 因此这里只需要将弧度映射到以 2047 为中心的逻辑坐标系。
*
* @param rad 弧度值 (建议范围 [-π, +π])
* @return 对应的 Feetech 刻度值 [0, 4095]
*/
static inline float rad_to_ticks(float rad) {
    /* 将弧度转换为 ticks 偏移量 (相对于中位) */
    float ticks_offset = rad * (4096.0f / (float)(2 * M_PI));

    /* 加上中位基准，四舍五入到最近整数以减少量化误差 */
    float ticks = roundf(STS3215_POSITION_MID + ticks_offset);

    /* 限制在有效范围内 [0, 4095] */
    if (ticks < 0)
        ticks = 0;
    if (ticks > 4095)
        ticks = 4095;

    return ticks;
}

/**
* @brief Feetech 刻度转弧度
*
* 将 Feetech 刻度转换回弧度，以中位 (2047) 为基准。
*
* 校准后的坐标系：
*   - 2047 ticks → 0 rad (中位，对应校准时的机械零位)
*   - 4095 ticks → +π rad (顺时针 180°)
*   - 0 ticks → -π rad (逆时针 180°)
*
* @param ticks Feetech 刻度值 [0, 4095]
* @return 对应的弧度值 (范围约 [-π, +π])
*/
static inline float ticks_to_rad(float ticks) {
    /* 计算相对于中位的偏移量 */
    float ticks_offset = ticks - STS3215_POSITION_MID;

    /* 转换为弧度 */
    return ticks_offset * ((float)(2 * M_PI) / 4096.0f);
}

/**
* @brief manipulator 关节角(rad, 以中位为 0) 转 motor API 角度(rad, 0~2π)
*/
static inline float joint_rad_to_motor_rad(float joint_rad) {
    float motor_rad = joint_rad + (float)M_PI;

    while (motor_rad < 0.0f)
        motor_rad += (float)(2 * M_PI);
    while (motor_rad >= (float)(2 * M_PI))
        motor_rad -= (float)(2 * M_PI);

    return motor_rad;
}

/**
* @brief motor API 角度(rad, 0~2π) 转 manipulator 关节角(rad, 以中位为 0)
*/
static inline float motor_rad_to_joint_rad(float motor_rad) {
    float joint_rad = motor_rad - (float)M_PI;

    while (joint_rad < -(float)M_PI)
        joint_rad += (float)(2 * M_PI);
    while (joint_rad > (float)M_PI)
        joint_rad -= (float)(2 * M_PI);

    return joint_rad;
}

static bool so101_targets_reached(const struct so101_priv *p,
                                    const manip_joint_t *cur_joints) {
    if (!p || !cur_joints || cur_joints->count == 0)
        return false;

    for (int i = 0; i < cur_joints->count && i < SO101_ARM_MOTOR_COUNT; ++i) {
        float target_joint = motor_rad_to_joint_rad(p->target_cmds[i].pos_des);
        float err = fabsf(cur_joints->joints[i] - target_joint);
        if (err > (float)M_PI)
            err = (float)(2 * M_PI) - err;
        if (err > SO101_REACHED_EPS_RAD)
            return false;
    }

    return true;
}

/* ==========================================================================
* Driver Ops
* ========================================================================== */

/**
* @brief 关节运动 — 将目标关节角下发给各舵机
*
* 把用户给的弧度值转为 Feetech 刻度，设置速度后批量下发。
* 调用后状态切换为 MANIP_MOVING。
*
* @param dev         设备句柄
* @param target      目标关节角 (弧度)，count 字段指定关节数
* @param speed_ratio 速度倍率 [0.0~1.0]，乘以默认速度 600
* @return MANIP_OK 成功，MANIP_ERR_PARAM 参数错误
*/
static int so101_move_joints(struct manip_dev *dev,
                            const manip_joint_t *target,
                            float speed_ratio) {
    if (!dev || !target)
        return MANIP_ERR_PARAM;

    struct so101_priv *p = (struct so101_priv *)dev->priv_data;
    uint8_t count = target->count;
    if (count > SO101_ARM_MOTOR_COUNT)
        count = SO101_ARM_MOTOR_COUNT;

    struct motor_cmd cmds[SO101_ARM_MOTOR_COUNT];
    memset(cmds, 0, sizeof(cmds));

    for (int i = 0; i < count; ++i) {
        cmds[i].mode = MOTOR_MODE_POS;
        cmds[i].pos_des = joint_rad_to_motor_rad(target->joints[i]);
        cmds[i].vel_des =
            SO101_DEFAULT_VEL_RAD_S *
                (speed_ratio > 0.0f ? speed_ratio : 1.0f);
    }

    /* 保存目标命令，供 tick() 持续发送 */
    memcpy(p->target_cmds, cmds, sizeof(cmds));
    p->has_target = true;

    /* 先设置状态，再发送命令 */
    pthread_mutex_lock(&dev->state_lock);
    dev->state = MANIP_MOVING;
    pthread_mutex_unlock(&dev->state_lock);

    int ret = motor_set_cmds(p->motors, cmds, count);
    if (ret != 0) {
        /* 通信失败：回滚状态，避免状态机与硬件不一致 */
        pthread_mutex_lock(&dev->state_lock);
        dev->state = MANIP_IDLE;
        pthread_mutex_unlock(&dev->state_lock);
        p->has_target = false;
        return MANIP_ERR_CONNECT;
    }
    return MANIP_OK;
}

/**
* @brief 急停 — 关闭所有关节扭矩
*
* 直接写 REG_TORQUE_ENABLE=0 释放扭矩，
* 不走 motor_set_cmds (MOTOR_MODE_IDLE 在 Feetech
* 驱动中会发送 position=0 导致机械臂突然运动)。
*
* @param dev 设备句柄
*/
static void so101_stop(struct manip_dev *dev) {
    if (!dev)
        return;
    struct so101_priv *p = (struct so101_priv *)dev->priv_data;

    /* 直接关闭每个舵机的扭矩 */
    so101_disable_torque_all(p->motors, SO101_ARM_MOTOR_COUNT);

    /* 清除目标命令和直线轨迹 */
    p->has_target = false;
    p->line.total = 0;
    p->line.current = 0;

    pthread_mutex_lock(&dev->state_lock);
    dev->state = MANIP_IDLE;
    pthread_mutex_unlock(&dev->state_lock);
}

/**
* @brief 获取当前关节状态 — 从舵机读取实际位置
*
* 批量读取 5 个舵机的实际关节角（rad），存入 dev->cur_joints。
* FK（正运动学）未实现，out_pose 返回全零。
*
* @param dev        设备句柄
* @param out_joints [输出] 当前关节角 (弧度)，可为 NULL
* @param out_pose   [输出] 末端位姿，目前未实现，可为 NULL
* @return MANIP_OK 成功，MANIP_ERR_CONNECT 通信失败
*/
static int so101_get_state(struct manip_dev *dev,
                            manip_joint_t *out_joints,
                            manip_pose_t *out_pose) {
    if (!dev)
        return MANIP_ERR_PARAM;

    struct so101_priv *p = (struct so101_priv *)dev->priv_data;
    struct motor_state states[SO101_ARM_MOTOR_COUNT];

    if (motor_get_states(p->motors, states, SO101_ARM_MOTOR_COUNT) != 0)
        return MANIP_ERR_CONNECT;

    pthread_mutex_lock(&dev->state_lock);

    dev->cur_joints.count = SO101_ARM_MOTOR_COUNT;
    for (int i = 0; i < SO101_ARM_MOTOR_COUNT; ++i)
        dev->cur_joints.joints[i] = motor_rad_to_joint_rad(states[i].pos);

    if (out_joints)
        *out_joints = dev->cur_joints;
    if (out_pose) {
        /* FK 由框架层完成：manipulator.c 的 manip_get_state()
            * 会通过 kin_forward() 计算末端位姿并填充 out_pose。
            * 驱动层不需要自己实现 FK。 */
        memset(out_pose, 0, sizeof(*out_pose));
    }

    pthread_mutex_unlock(&dev->state_lock);
    return MANIP_OK;
}

/**
* @brief 周期更新 — 持续发送运动命令并刷新状态
*
* Feetech 舵机需要持续接收命令才能保持运动，
* 因此在 MOVING 状态下，每次 tick 都重新发送目标位置。
* 应用层应以固定频率 (如 10Hz) 调用此函数。
*
* @param dev  设备句柄
* @param dt_s 距上次 tick 的时间间隔 (秒)，当前未使用
*/
static void so101_tick(struct manip_dev *dev, float dt_s) {
    (void)dt_s;
    if (!dev)
        return;

    struct so101_priv *p = (struct so101_priv *)dev->priv_data;

    if (dev->state == MANIP_MOVING) {
        /* 直线轨迹模式：逐步推进 waypoint */
        if (p->line.total > 0) {
            int idx = p->line.current;
            motor_set_cmds(p->motors,
                                            p->line.steps[idx],
                                            SO101_ARM_MOTOR_COUNT);
            if (idx + 1 < p->line.total) {
                p->line.current = idx + 1;
            } else {
                /* 轨迹结束：切换为 PTP 保持最后 waypoint */
                memcpy(p->target_cmds,
                                p->line.steps[idx],
                                sizeof(p->target_cmds));
                p->has_target = true;
                p->line.total = 0;
            }
        } else if (p->has_target) {
            /* PTP 模式：持续发送目标 */
            motor_set_cmds(p->motors,
                                            p->target_cmds,
                                            SO101_ARM_MOTOR_COUNT);
        }
    }

    /* Refresh state from hardware */
    if (so101_get_state(dev, NULL, NULL) != MANIP_OK)
        return;

    if (dev->state == MANIP_MOVING && p->line.total == 0 && p->has_target) {
        pthread_mutex_lock(&dev->state_lock);
        if (so101_targets_reached(p, &dev->cur_joints)) {
            dev->state = MANIP_IDLE;
        }
        pthread_mutex_unlock(&dev->state_lock);
    }
}

/**
* @brief 释放 SO-101 实例 — 关闭所有舵机并释放内存
*
* 先释放 5 个 motor_dev，再调用 manip_dev_free_default 释放设备结构体。
* priv_data 由 manip_dev_free_default 统一释放，这里不要手动 free。
*
* @param dev 设备句柄
*/
static void so101_free(struct manip_dev *dev) {
    if (!dev)
        return;

    struct so101_priv *p = (struct so101_priv *)dev->priv_data;
    if (p) {
        motor_free(p->motors, SO101_ARM_MOTOR_COUNT);
        /* priv_data + kin_solver freed by manip_dev_free_default */
    }
    manip_dev_free_default(dev);
}

/**
* @brief 示教模式 — 使能/关闭扭矩
*
* enable=true: 关闭扭矩 (舵机可自由拖动 — 示教模式)
* enable=false: 使能扭矩 (舵机锁定 — 正常模式)
*
* @param dev    设备句柄
* @param enable true=进入示教模式(关闭扭矩), false=退出示教模式(使能扭矩)
* @return MANIP_OK 成功
*/
static int so101_set_teach_mode(struct manip_dev *dev, bool enable) {
    if (!dev)
        return MANIP_ERR_PARAM;
    struct so101_priv *p = (struct so101_priv *)dev->priv_data;

    int ret;
    if (enable) {
        ret = so101_disable_torque_all(p->motors, SO101_ARM_MOTOR_COUNT);
        pthread_mutex_lock(&dev->state_lock);
        dev->state = MANIP_IDLE;
        pthread_mutex_unlock(&dev->state_lock);
    } else {
        ret = so101_enable_torque_all(p->motors, SO101_ARM_MOTOR_COUNT);
    }

    return (ret == 0) ? MANIP_OK : MANIP_ERR_CONNECT;
}

/* ==========================================================================
* move_line — 笛卡尔直线插补
* ========================================================================== */

/**
* @brief 笛卡尔直线运动 (TCP 空间插补)
*
* 1. FK 获取当前 TCP 位姿 (起点)
* 2. 按距离计算插补段数 N = dist / step
* 3. 在笛卡尔空间线性插值 N 个 waypoint
* 4. 对每个 waypoint 做 IK，存入轨迹队列
* 5. tick() 逐个发送
*
* 需要 kin_solver 已绑定，否则返回 MANIP_ERR_NOSYS。
*
* @param dev         设备句柄
* @param target      目标末端位姿 (位置 m, 姿态四元数)
* @param speed_ratio 速度倍率 (0~1)
* @return MANIP_OK 成功, MANIP_ERR_NOSYS 无求解器
*/
static int so101_move_line(struct manip_dev *dev,
                            const manip_pose_t *target,
                            float speed_ratio) {
    if (!dev || !target)
        return MANIP_ERR_PARAM;

    /* 需要 IK 求解器 */
    if (!dev->kin_solver)
        return MANIP_ERR_NOSYS;

    struct so101_priv *p = (struct so101_priv *)dev->priv_data;

    /* --- 1. FK: 获取当前末端位姿 --- */
    kin_joints_t q_cur;
    q_cur.count = (uint8_t)SO101_ARM_MOTOR_COUNT;
    for (int i = 0; i < SO101_ARM_MOTOR_COUNT; i++)
        q_cur.q[i] = (double)dev->cur_joints.joints[i];

    kin_pose_t start;
    if (kin_forward(dev->kin_solver, &q_cur, &start) != KIN_OK) {
        fprintf(stderr, "[SO101] move_line: FK failed\n");
        return MANIP_ERR_NOSYS;
    }

    /* --- 2. 计算插补段数 --- */
    double dx = target->x - start.x;
    double dy = target->y - start.y;
    double dz = target->z - start.z;
    double dist = sqrt(dx * dx + dy * dy + dz * dz);

    int steps = (int)ceil(dist / SO101_LINE_STEP_M);
    if (steps < 1)
        steps = 1;
    if (steps > SO101_LINE_MAX_STEPS)
        steps = SO101_LINE_MAX_STEPS;

    /* --- 3+4. 插值 + IK --- */
    kin_joints_t q_seed = q_cur; /* IK 种子 */
    float vel = SO101_DEFAULT_VEL_RAD_S *
                            (speed_ratio > 0.0f ? speed_ratio : 1.0f);

    for (int s = 0; s < steps; s++) {
        double t = (double)(s + 1) / (double)steps;

        /* 笛卡尔线性插值 */
        kin_pose_t wp;
        wp.x  = start.x + dx * t;
        wp.y  = start.y + dy * t;
        wp.z  = start.z + dz * t;
        /* 姿态：简单线性混合四元数 (LERP + normalize) */
        wp.qw = start.qw * (1.0 - t) + target->qw * t;
        wp.qx = start.qx * (1.0 - t) + target->qx * t;
        wp.qy = start.qy * (1.0 - t) + target->qy * t;
        wp.qz = start.qz * (1.0 - t) + target->qz * t;
        double qn = sqrt(wp.qw * wp.qw + wp.qx * wp.qx +
                                            wp.qy * wp.qy + wp.qz * wp.qz);
        if (qn > 1e-9) {
            wp.qw /= qn; wp.qx /= qn;
            wp.qy /= qn; wp.qz /= qn;
        }

        /* 仅位置 IK (5-DOF 推荐) */
        kin_ik_params_t ik_params = {
                .epsilon = 1e-3,
                .position_weight = 1.0,
                .timeout_s = 0.2,
        };
        kin_joints_t q_out;
        int ik_ret = kin_inverse(dev->kin_solver,
                                                            &wp, &q_seed,
                                                            &ik_params, &q_out);
        if (ik_ret != KIN_OK) {
            fprintf(stderr,
                            "[SO101] move_line: IK failed at step %d/%d\n",
                            s + 1, steps);
            return MANIP_ERR_NOSYS;
        }

        /* 转换为 motor_cmd 并存入轨迹 */
        for (int j = 0; j < SO101_ARM_MOTOR_COUNT; j++) {
            p->line.steps[s][j].mode = MOTOR_MODE_POS;
                p->line.steps[s][j].pos_des =
                    joint_rad_to_motor_rad((float)q_out.q[j]);
            p->line.steps[s][j].vel_des = vel;
        }

        /* 当前解作为下一段的种子 (连续性) */
        q_seed = q_out;
    }

    /* --- 5. 启动轨迹执行 --- */
    p->line.total = steps;
    p->line.current = 0;
    p->has_target = false;

    pthread_mutex_lock(&dev->state_lock);
    dev->state = MANIP_MOVING;
    pthread_mutex_unlock(&dev->state_lock);

    return MANIP_OK;
}

/**
* @brief SO-101 驱动操作表
*/
static const struct manip_ops so101_ops = {
        .move_joints = so101_move_joints,
        .move_line = so101_move_line,
        .stop = so101_stop,
        .set_tcp = NULL,
        .set_teach_mode = so101_set_teach_mode,
        .get_state = so101_get_state,
        .tick = so101_tick,
        .free = so101_free,
};

/* ==========================================================================
* Factory Function
* ========================================================================== */

/**
* @brief 工厂函数 — 创建一个 SO-101 机械臂实例
*
* 由框架在 manip_alloc("so101", args) 时调用。
* 流程：
*   1. 解析配置 (args)，或使用默认值
*   2. 分配 manip_dev + so101_priv 内存
*   3. 为 5 个关节分别创建 motor_dev (Feetech UART)
*   4. 批量初始化所有电机
*   5. 挂载操作表，标记 running=true
*
* 波特率编码规则 (motor 库约定)：
*   高 24 位 = 实际波特率，低 8 位 = 舵机 ID
*   例如 1000000 | 3 → 波特率 1M, 舵机 ID=3
*
* @param name 驱动名 (由框架传入，即 "so101")
* @param args 配置参数指针 (struct so101_config*)，可为 NULL 使用默认值
* @return 成功返回设备句柄，失败返回 NULL
*/
static struct manip_dev *so101_factory(const char *name, void *args) {
    struct so101_config cfg;

    if (args) {
        memcpy(&cfg, args, sizeof(cfg));
    } else {
        /* Defaults: /dev/ttyACM0, 1Mbaud, IDs 1-5 */
        cfg.uart_path = "/dev/ttyACM0";
        cfg.baud = 1000000;
        for (int i = 0; i < SO101_ARM_MOTOR_COUNT; ++i)
            cfg.ids[i] = (uint8_t)(i + 1);
        cfg.urdf_path = NULL;
        cfg.kin_solver_name = NULL;
    }

    /* URDF 路径: 配置传入 > 默认路径 */
    const char *urdf = cfg.urdf_path ? cfg.urdf_path : SO101_DEFAULT_URDF_PATH;

    struct manip_dev *dev =
            manip_dev_alloc(name, sizeof(struct so101_priv));
    if (!dev)
        return NULL;

    struct so101_priv *p = (struct so101_priv *)dev->priv_data;

    /* Allocate motors */
    for (int i = 0; i < SO101_ARM_MOTOR_COUNT; ++i) {
        p->motors[i] =
                motor_alloc_uart("drv_uart_feetech",
                                    cfg.uart_path,
                                    cfg.baud,
                                    cfg.ids[i],
                                    NULL);
        if (!p->motors[i]) {
            fprintf(stderr, "[SO101] Failed to alloc arm motor %d (ID=%u)\n", i,
                            cfg.ids[i]);
            motor_free(p->motors, (uint32_t)i);
            manip_dev_free_default(dev);
            return NULL;
        }
    }

    if (motor_init(p->motors, SO101_ARM_MOTOR_COUNT) != 0) {
        fprintf(stderr, "[SO101] motor_init failed\n");
        motor_free(p->motors, SO101_ARM_MOTOR_COUNT);
        manip_dev_free_default(dev);
        return NULL;
    }

    /* 配置舵机参数 (PID/加速度/延迟) — 每次启动都配置以确保一致性 */
    so101_disable_torque_all(p->motors, SO101_ARM_MOTOR_COUNT);
    so101_configure_motors(p->motors, SO101_ARM_MOTOR_COUNT);

    /* 尝试加载并应用校准数据 */
    memset(&p->calib, 0, sizeof(p->calib));
    if (so101_load_calibration(&p->calib, SO101_CALIBRATION_PATH) == 0) {
        so101_write_calibration_to_motors(p->motors, &p->calib);
    } else {
        printf("[SO101] 未找到校准文件，使用默认参数\n");
        printf("[SO101] 建议执行 assemble + calibrate 流程\n");
    }

    /* 使能扭矩 */
    so101_enable_torque_all(p->motors, SO101_ARM_MOTOR_COUNT);
    p->assembled = true;
    p->has_target = false;  /* 初始化运动控制状态 */
    dev->cur_joints.count = SO101_ARM_MOTOR_COUNT;

    /* 如果加载了校准数据，发送命令让关节移动到零位 (0 rad) */
    if (p->calib.valid) {
        struct motor_cmd cmds[SO101_ARM_MOTOR_COUNT];
        for (int i = 0; i < SO101_ARM_MOTOR_COUNT; i++) {
            cmds[i].mode = MOTOR_MODE_POS;
            cmds[i].pos_des = joint_rad_to_motor_rad(0.0f);
            cmds[i].vel_des = SO101_DEFAULT_VEL_RAD_S;
        }
        motor_set_cmds(p->motors, cmds, SO101_ARM_MOTOR_COUNT);
        printf("[SO101] 已发送归零命令 (关节目标: 0 rad)\n");
    }

    /* 同步一次初始状态，避免 move_line/FK 使用未初始化关节角 */
    so101_get_state(dev, NULL, NULL);

    dev->ops = &so101_ops;
    dev->running = true;

    /* 尝试自动绑定运动学求解器 (IK/FK) */
    {
        kin_solver_t *kin = kin_create(cfg.kin_solver_name,
                                        urdf,
                                        SO101_BASE_LINK,
                                        SO101_TIP_LINK);
        if (kin) {
            dev->kin_solver = kin;
            printf("[SO101] Kinematics solver attached (FK/IK enabled)\n");
        } else {
            printf("[SO101] No kinematics solver available "
                    "(move_line disabled)\n");
        }
    }

    return dev;
}

/**
* @brief 获取关节名称
*/
const char *so101_joint_name(int index) {
    if (index >= 0 && index < SO101_ARM_MOTOR_COUNT)
        return so101_joint_names[index];
    return "unknown";
}

/**
* @brief 驱动自动注册
*
* 利用 GCC __attribute__((constructor))，在程序启动时
* 自动将 "so101" 驱动注册到 manipulator 框架的驱动链表中。
* 用户只需 link 这个 .o 文件，无需手动调用注册函数。
*/
REGISTER_MANIP_DRIVER("so101", so101_factory)

#endif /* HAVE_MOTOR */
