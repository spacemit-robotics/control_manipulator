/*
* Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
* SPDX-License-Identifier: Apache-2.0
*
* @file test_hw_so101.c
* @brief SO-101 机械臂 + 夹爪 硬件测试程序
*
* 接上真实硬件后，通过交互式菜单测试各项功能。
* 默认配置: /dev/ttyACM0, 1Mbaud, 关节 ID 1-5, 夹爪 ID 6
*
* 编译 (需要 motor 库):
*   rm build && mkdir build && cd build
*   cmake -DMANIP_BUILD_HW_TEST=ON .. && make
*
* 运行:
*   sudo ./test_hw_so101              # 使用默认配置
*   sudo ./test_hw_so101 /dev/ttyUSB0 # 指定串口
*
* ⚠️ 注意: 机械臂会实际运动，请确保周围安全！
*/

#include <math.h>
#include <signal.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "grasp.h"
#include "kinematics_interface.h"
#include "manipulator.h"
#include "motor.h"
#include "so101_utils.h"
#include "sts3215_regs.h"

/* --- Register access helpers (wrappers for motor_set/get_paras) --- */

static inline int reg_read_byte(struct motor_dev *dev,
                                                                uint8_t reg) {
    uint8_t val = 0;
    if (motor_get_paras(dev, &reg, &val, 1) != 0)
        return -1;
    return (int)val;
}

static inline int reg_read_word(struct motor_dev *dev,
                                                                uint8_t reg) {
    uint16_t val = 0;
    if (motor_get_paras(dev, &reg, &val, 2) != 0)
        return -1;
    return (int)val;
}

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ========================================================================== */
/* 全局句柄 (方便信号处理函数访问) */
/* ========================================================================== */
static struct manip_dev *g_arm = NULL;
static struct grasp_dev *g_grip = NULL;
static kin_solver_t *g_kin_solver = NULL;
static volatile int g_running = 1;
static const char *g_uart_path = "/dev/ttyACM0";

/* Ctrl+C 信号处理：急停并退出 */
static void signal_handler(int sig) {
    (void)sig;
    printf("\n[!] 收到中断信号，正在急停...\n");
    g_running = 0;
}

/* ========================================================================== */
/* 辅助函数 */
/* ========================================================================== */

/* 度数转弧度 */
static float deg2rad(float deg) { return deg * (float)M_PI / 180.0f; }

/* 弧度转度数 */
static float rad2deg(float rad) { return rad * 180.0f / (float)M_PI; }

/* 打印当前关节角度 */
static void print_joints(const manip_joint_t *j) {
    printf("  关节角度 (共 %d 轴):\n", j->count);
    for (int i = 0; i < j->count; i++) {
        printf("    [%d] %.2f rad  (%.1f°)\n", i + 1, j->joints[i],
                        rad2deg(j->joints[i]));
    }
}

/* 打印末端位姿 */
static void print_pose(const manip_pose_t *p) {
    printf("  末端位姿:\n");
    printf("    位置: x=%.4f m, y=%.4f m, z=%.4f m\n", p->x, p->y, p->z);
    printf("    姿态: qw=%.4f, qx=%.4f, "
            "qy=%.4f, qz=%.4f\n",
                    p->qw, p->qx, p->qy, p->qz);
    /* 计算欧拉角 (简化显示，仅供参考) */
    double roll = atan2(2.0 * (p->qw * p->qx + p->qy * p->qz),
                        1.0 - 2.0 * (p->qx * p->qx + p->qy * p->qy));
    double pitch = asin(2.0 * (p->qw * p->qy - p->qz * p->qx));
    double yaw = atan2(2.0 * (p->qw * p->qz + p->qx * p->qy),
                        1.0 - 2.0 * (p->qy * p->qy + p->qz * p->qz));
    float roll_deg = rad2deg((float)roll);
    float pitch_deg = rad2deg((float)pitch);
    float yaw_deg = rad2deg((float)yaw);
    printf("    欧拉角: roll=%.1f°, pitch=%.1f°, yaw=%.1f°\n",
            roll_deg, pitch_deg, yaw_deg);
}

/* 打印夹爪状态 */
static void print_grasp_state(struct grasp_dev *grip) {
    grasp_state_t s = grasp_get_state(grip);
    const char *names[] = {"IDLE", "MOVING", "HOLDING", "EMPTY", "ERROR"};
    int idx = (int)s;
    if (idx < 0 || idx > 4)
        idx = 4;
    printf("  夹爪状态: %s\n", names[idx]);

    float pos = 0, load = 0;
    if (grasp_get_feedback(grip, &pos, &load) == GRASP_OK) {
        printf("  夹爪位置: %.1f%%  负载: %.1f\n", pos * 100.0f, load);
    }
}

static struct manip_dev *alloc_arm_on_current_port(void) {
    struct so101_config arm_cfg = {
        .uart_path = g_uart_path,
        .baud = 1000000,
        .ids = {1, 2, 3, 4, 5},
        .urdf_path = NULL,
        .kin_solver_name = NULL,
    };

    return manip_alloc("so101", &arm_cfg);
}

static struct grasp_dev *alloc_grip_on_current_port(void) {
    struct so101_gripper_config grip_cfg = {
        .uart_path = g_uart_path,
        .baud = 1000000,
        .id = 6,
        .grasp_cfg = {
            .max_effort = 1.0f,
            .hold_threshold = 100.0f,
            .timeout_ms = 5000,
        },
    };

    return grasp_alloc("so101_gripper", &grip_cfg);
}

/* 等待用户按回车 */
static void wait_enter(void) {
    printf("  按 Enter 继续...");
    while (getchar() != '\n') {}
}

/* 清空 stdin 多余字符 */
static void flush_stdin(void) {
    int c;
    while ((c = getchar()) != '\n' && c != EOF) {
        continue;
    }
}

/* ========================================================================== */
/* 测试项 */
/* ========================================================================== */

/**
* 测试 1: 读取当前关节角度
*/
static void test_read_state(void) {
    printf("\n=== 测试 1: 读取关节状态 ===\n");

    manip_joint_t joints;
    int ret = manip_get_state(g_arm, &joints, NULL);
    if (ret != MANIP_OK) {
        printf("  [错误] 读取失败 (err=%d)，检查串口连接\n", ret);
        return;
    }
    print_joints(&joints);
    print_grasp_state(g_grip);
}

/**
* 测试 2: 单轴运动 — 选择一个关节，转动指定角度
*/
static void test_single_joint(void) {
    printf("\n=== 测试 2: 单轴运动 ===\n");

    /* 先读当前位置 */
    manip_joint_t cur;
    if (manip_get_state(g_arm, &cur, NULL) != MANIP_OK) {
        printf("  [错误] 无法读取当前位置\n");
        return;
    }
    print_joints(&cur);

    printf("  选择关节 (1-5): ");
    int joint_id = 0;
    if (scanf("%d", &joint_id) != 1 || joint_id < 1 || joint_id > 5) {
        printf("  [错误] 无效关节号\n");
        flush_stdin();
        return;
    }

    printf("  输入目标角度 (度): ");
    float deg = 0;
    if (scanf("%f", &deg) != 1) {
        printf("  [错误] 无效输入\n");
        flush_stdin();
        return;
    }
    flush_stdin();

    /* 以当前位置为基础，只修改选中的关节 */
    manip_joint_t target = cur;
    target.joints[joint_id - 1] = deg2rad(deg);

    printf("  → 关节 %d 目标: %.1f° (速度 50%%)\n", joint_id, deg);
    printf("  ⚠️ 机械臂即将运动！确认？(y/n): ");
    char c = 0;
    if (scanf("%c", &c) != 1 || (c != 'y' && c != 'Y')) {
        printf("  已取消\n");
        flush_stdin();
        return;
    }
    flush_stdin();

    int ret = manip_move_joints(g_arm, &target, 0.5f);
    if (ret != MANIP_OK) {
        printf("  [错误] 发送失败 (err=%d)\n", ret);
        return;
    }

    printf("  运动中... 等待 2 秒\n");
    for (int i = 0; i < 20 && g_running; i++) {
        usleep(100000); /* 100ms */
        manip_tick(g_arm, 0.1f);
    }

    /* 读取到达后的位置 */
    manip_joint_t after;
    if (manip_get_state(g_arm, &after, NULL) == MANIP_OK) {
        printf("  运动完成，当前位置:\n");
        print_joints(&after);
    }
}

/**
* 测试 3: 所有关节归零 (回到 0 度)
*/
static void test_home(void) {
    printf("\n=== 测试 3: 关节归零 ===\n");
    printf("  ⚠️ 所有关节将回到 0° 位置！确认？(y/n): ");
    char c = 0;
    if (scanf("%c", &c) != 1 || (c != 'y' && c != 'Y')) {
        printf("  已取消\n");
        flush_stdin();
        return;
    }
    flush_stdin();

    manip_joint_t home = {.count = 5, .joints = {0}};
    int ret = manip_move_joints(g_arm, &home, 0.3f);
    if (ret != MANIP_OK) {
        printf("  [错误] 发送失败 (err=%d)\n", ret);
        return;
    }

    printf("  归零中... 等待 3 秒\n");
    for (int i = 0; i < 30 && g_running; i++) {
        usleep(100000);
        manip_tick(g_arm, 0.1f);
    }

    manip_joint_t after;
    if (manip_get_state(g_arm, &after, NULL) == MANIP_OK) {
        printf("  归零完成:\n");
        print_joints(&after);
    }
}

/**
* 测试 4: 预设动作序列 — 演示性的挥手动作
*/
static void test_wave(void) {
    printf("\n=== 测试 4: 挥手动作演示 ===\n");
    printf("  ⚠️ 机械臂将执行挥手动作！确认？(y/n): ");
    char c = 0;
    if (scanf("%c", &c) != 1 || (c != 'y' && c != 'Y')) {
        printf("  已取消\n");
        flush_stdin();
        return;
    }
    flush_stdin();

    /* 先抬起手臂 */
    manip_joint_t pose_up = {
            .count = 5,
            .joints = {0, deg2rad(45), deg2rad(-30), deg2rad(60), 0}};
    printf("  [1/4] 抬臂...\n");
    manip_move_joints(g_arm, &pose_up, 0.4f);
    for (int i = 0; i < 15 && g_running; i++) {
        usleep(100000);
        manip_tick(g_arm, 0.1f);
    }

    /* 挥手: 关节 1 左右摆 */
    for (int wave = 0; wave < 3 && g_running; wave++) {
        manip_joint_t wave_left = pose_up;
        wave_left.joints[0] = deg2rad(-30);
        printf("  [挥手] ← \n");
        manip_move_joints(g_arm, &wave_left, 0.6f);
        for (int i = 0; i < 8 && g_running; i++) {
            usleep(100000);
            manip_tick(g_arm, 0.1f);
        }

        manip_joint_t wave_right = pose_up;
        wave_right.joints[0] = deg2rad(30);
        printf("  [挥手] → \n");
        manip_move_joints(g_arm, &wave_right, 0.6f);
        for (int i = 0; i < 8 && g_running; i++) {
            usleep(100000);
            manip_tick(g_arm, 0.1f);
        }
    }

    /* 回中 */
    printf("  [4/4] 回到初始...\n");
    manip_joint_t home = {.count = 5, .joints = {0}};
    manip_move_joints(g_arm, &home, 0.3f);
    for (int i = 0; i < 20 && g_running; i++) {
        usleep(100000);
        manip_tick(g_arm, 0.1f);
    }
    printf("  挥手完成!\n");
}

/**
* 测试 5: 夹爪开合
*/
static void test_gripper(void) {
    printf("\n=== 测试 5: 夹爪控制 ===\n");
    print_grasp_state(g_grip);

    printf("  选择操作:\n");
    printf("    1. 夹取 (GRAB)\n");
    printf("    2. 松开 (RELEASE)\n");
    printf("    3. 放松 (RELAX / 掉电)\n");
    printf("    4. 移动到指定位置\n");
    printf("  输入 (1-4): ");

    int choice = 0;
    if (scanf("%d", &choice) != 1 || choice < 1 || choice > 4) {
        printf("  [错误] 无效输入\n");
        flush_stdin();
        return;
    }
    flush_stdin();

    int ret = GRASP_OK;
    switch (choice) {
    case 1:
        printf("  输入力度 [0.0~1.0] (推荐 0.5): ");
        float effort = 0.5f;
        if (scanf("%f", &effort) != 1)
            effort = 0.5f;
        flush_stdin();
        printf("  → 执行夹取 (力度 %.1f)...\n", effort);
        ret = grasp_execute(g_grip, GRASP_CMD_GRAB, effort);
        break;

    case 2:
        printf("  → 执行松开...\n");
        ret = grasp_execute(g_grip, GRASP_CMD_RELEASE, 0.5f);
        break;

    case 3:
        printf("  → 执行放松 (舵机掉电)...\n");
        ret = grasp_execute(g_grip, GRASP_CMD_RELAX, 0.0f);
        break;

    case 4: {
        printf("  输入目标位置 [0.0=全闭, 1.0=全开]: ");
        float pos = 0.5f;
        if (scanf("%f", &pos) != 1)
            pos = 0.5f;
        flush_stdin();
        printf("  → 移动到 %.0f%%...\n", pos * 100);
        ret = grasp_set_position(g_grip, pos);
        break;
    }
    }

    if (ret != GRASP_OK) {
        printf("  [错误] 操作失败 (err=%d)\n", ret);
        return;
    }

    /* 等待动作完成 */
    printf("  等待中...\n");
    for (int i = 0; i < 20 && g_running; i++) {
        usleep(100000);
        grasp_tick(g_grip, 0.1f);
    }
    print_grasp_state(g_grip);
}

/**
* 测试 6: 急停
*/
static void test_stop(void) {
    printf("\n=== 急停 ===\n");
    manip_stop(g_arm);
    grasp_execute(g_grip, GRASP_CMD_RELAX, 0.0f);
    printf("  已急停，所有舵机释放扭矩\n");
}

/**
* 测试 7: 连续读取状态 (示波器模式)
*/
static void test_monitor(void) {
    printf("\n=== 测试 7: 状态监控 (按 Ctrl+C 停止) ===\n");
    printf("  %-6s", "时间");
    for (int i = 1; i <= 5; i++)
        printf("  关节%d(°)  ", i);
    printf("  夹爪(%%)  负载\n");
    printf("  %-6s", "----");
    for (int i = 0; i < 5; i++)
        printf("  --------  ");
    printf("  --------  ----\n");

    float t = 0;
    while (g_running) {
        manip_joint_t joints;
        if (manip_get_state(g_arm, &joints, NULL) == MANIP_OK) {
            printf("  %5.1fs", t);
            for (int i = 0; i < 5; i++)
                printf("  %8.1f  ", rad2deg(joints.joints[i]));

            float pos = 0, load = 0;
            grasp_get_feedback(g_grip, &pos, &load);
            printf("  %6.1f%%  %5.1f", pos * 100, load);
            printf("\n");
        }
        usleep(200000); /* 200ms = 5Hz */
        t += 0.2f;
        manip_tick(g_arm, 0.2f);
        grasp_tick(g_grip, 0.2f);
    }
    /* 恢复运行标志 (如果是在菜单中使用) */
    g_running = 1;
}

/**
* 测试 8: 手动输入 5 轴角度
*/
static void test_manual_move(void) {
    printf("\n=== 测试 8: 手动输入关节角 ===\n");

    manip_joint_t cur;
    if (manip_get_state(g_arm, &cur, NULL) == MANIP_OK) {
        printf("  当前位置:\n");
        print_joints(&cur);
    }

    manip_joint_t target = {.count = 5};
    printf("  请依次输入 5 个关节角 (度)，用空格分隔:\n  > ");
    for (int i = 0; i < 5; i++) {
        float deg = 0;
        if (scanf("%f", &deg) != 1) {
            printf("  [错误] 输入格式有误\n");
            flush_stdin();
            return;
        }
        target.joints[i] = deg2rad(deg);
    }
    flush_stdin();

    printf("  输入速度倍率 [0.1~1.0] (默认 0.3): ");
    float speed = 0.3f;
    char line[64];
    if (fgets(line, sizeof(line), stdin) && line[0] != '\n') {
        speed = strtof(line, NULL);
        if (speed < 0.05f)
            speed = 0.05f;
        if (speed > 1.0f)
            speed = 1.0f;
    }

    printf("  目标:");
    for (int i = 0; i < 5; i++)
        printf(" %.1f°", rad2deg(target.joints[i]));
    printf("  速度: %.0f%%\n", speed * 100);

    printf("  ⚠️ 确认运动？(y/n): ");
    char c = 0;
    if (scanf("%c", &c) != 1 || (c != 'y' && c != 'Y')) {
        printf("  已取消\n");
        flush_stdin();
        return;
    }
    flush_stdin();

    int ret = manip_move_joints(g_arm, &target, speed);
    if (ret != MANIP_OK) {
        printf("  [错误] 发送失败 (err=%d)\n", ret);
        return;
    }

    printf("  运动中...\n");
    for (int i = 0; i < 30 && g_running; i++) {
        usleep(100000);
        manip_tick(g_arm, 0.1f);
    }

    if (manip_get_state(g_arm, &cur, NULL) == MANIP_OK) {
        printf("  到达位置:\n");
        print_joints(&cur);
    }
}

/**
* 测试 9: 组装 (Assemble) — 配置舵机参数
*
* 独立于 manip_alloc 的流程，直接操作 motor_dev 层。
* 用于首次组装、更换舵机后重新配置。
*
* ⚠️ 会暂时释放机械臂和夹爪句柄，完成后重新初始化。
*/
static void test_assemble(void) {
    printf("\n=== 测试 9: 组装 (Assemble) ===\n");
    printf("  此过程将:\n");
    printf("    1. 释放当前机械臂/夹爪连接\n");
    printf("    2. 关闭所有关节扭矩 (舵机可自由转动)\n");
    printf("    3. 配置 PID (P=%d I=%d D=%d)\n",
                    STS3215_SO101_P, STS3215_SO101_I, STS3215_SO101_D);
    printf("    4. 配置加速度 (%d) 和延迟 (0)\n", STS3215_SO101_ACC);
    printf("    5. 完成后重新初始化\n");
    printf("\n  ⚠️  确认？(y/n): ");

    char c = 0;
    if (scanf("%c", &c) != 1 || (c != 'y' && c != 'Y')) {
        printf("  已取消\n");
        flush_stdin();
        return;
    }
    flush_stdin();

    /* 释放现有连接，避免与新建的 motor_dev 冲突 */
    printf("  释放现有连接...\n");
    if (g_arm) {
        manip_stop(g_arm);
        manip_free(g_arm);
        g_arm = NULL;
    }
    if (g_grip) {
        grasp_execute(g_grip, GRASP_CMD_RELAX, 0.0f);
        grasp_free(g_grip);
        g_grip = NULL;
    }

    /* 创建临时电机句柄 */
    struct motor_dev *motors[SO101_ARM_MOTOR_COUNT];

    for (int i = 0; i < SO101_ARM_MOTOR_COUNT; i++) {
        motors[i] = motor_alloc_uart(
                "drv_uart_feetech", g_uart_path, 1000000,
                (uint8_t)(i + 1), NULL);
        if (!motors[i]) {
            fprintf(stderr, "  [错误] 分配电机 %d 失败\n", i + 1);
            motor_free(motors, (uint32_t)i);
            goto reinit;
        }
    }

    if (motor_init(motors, SO101_ARM_MOTOR_COUNT) != 0) {
        fprintf(stderr, "  [错误] 电机初始化失败\n");
        motor_free(motors, SO101_ARM_MOTOR_COUNT);
        goto reinit;
    }

    /* 执行组装 */
    if (so101_assemble(motors, SO101_ARM_MOTOR_COUNT) == 0) {
        printf("\n  ✓ 组装完成!\n");
        printf("  现在可以手动将各关节摆到中间位置，然后执行 '校准'\n");
    } else {
        printf("\n  ✗ 组装失败\n");
    }

    motor_free(motors, SO101_ARM_MOTOR_COUNT);

reinit:
    /* 重新初始化机械臂和夹爪 */
    printf("\n  重新初始化机械臂...\n");
    g_arm = alloc_arm_on_current_port();
    if (g_arm)
        printf("  ✓ 机械臂就绪\n");
    else
        printf("  ✗ 机械臂初始化失败\n");

    printf("  重新初始化夹爪...\n");
    g_grip = alloc_grip_on_current_port();
    if (g_grip)
        printf("  ✓ 夹爪就绪\n");
    else
        printf("  ✗ 夹爪初始化失败\n");
}

/**
* 测试 10: 校准 (Calibrate) — 记录中位偏移和行程范围
*
* ⚠️ 会暂时释放机械臂/夹爪句柄。
*/
static void test_calibrate(void) {
    printf("\n=== 测试 10: 校准 (Calibrate) ===\n");
    printf("  此过程将:\n");
    printf("    1. 释放当前连接并关闭所有扭矩\n");
    printf("    2. 请您手动将各关节移到中间位置\n");
    printf("    3. 记录中位偏移 (homing_offset)\n");
    printf("    4. (可选) 请您手动将各关节转到极限位置\n");
    printf("    5. 保存校准数据到 %s\n", SO101_DEFAULT_CALIBRATION_PATH);
    printf("\n  选择模式:\n");
    printf("    1. 交互模式 (手动移动关节到极限)\n");
    printf("    2. 快速模式 (使用默认行程范围)\n");
    printf("  输入 (1-2): ");

    int mode = 0;
    if (scanf("%d", &mode) != 1 || (mode != 1 && mode != 2)) {
        printf("  [错误] 无效输入\n");
        flush_stdin();
        return;
    }
    flush_stdin();

    bool interactive = (mode == 1);

    printf("\n  ⚠️  舵机将释放扭矩！确认开始？(y/n): ");
    char c = 0;
    if (scanf("%c", &c) != 1 || (c != 'y' && c != 'Y')) {
        printf("  已取消\n");
        flush_stdin();
        return;
    }
    flush_stdin();

    /* 释放现有连接 */
    printf("  释放现有连接...\n");
    if (g_arm) {
        manip_stop(g_arm);
        manip_free(g_arm);
        g_arm = NULL;
    }
    if (g_grip) {
        grasp_execute(g_grip, GRASP_CMD_RELAX, 0.0f);
        grasp_free(g_grip);
        g_grip = NULL;
    }

    /* 创建临时电机句柄 */
    struct motor_dev *motors[SO101_ARM_MOTOR_COUNT];

    for (int i = 0; i < SO101_ARM_MOTOR_COUNT; i++) {
        motors[i] = motor_alloc_uart(
                "drv_uart_feetech", g_uart_path, 1000000,
                (uint8_t)(i + 1), NULL);
        if (!motors[i]) {
            fprintf(stderr, "  [错误] 分配电机 %d 失败\n", i + 1);
            motor_free(motors, (uint32_t)i);
            goto reinit;
        }
    }

    if (motor_init(motors, SO101_ARM_MOTOR_COUNT) != 0) {
        fprintf(stderr, "  [错误] 电机初始化失败\n");
        motor_free(motors, SO101_ARM_MOTOR_COUNT);
        goto reinit;
    }

    struct so101_calibration calib;
    memset(&calib, 0, sizeof(calib));

    if (so101_calibrate(motors, &calib, interactive, NULL) == 0) {
        printf("\n  ✓ 校准完成!\n");
        printf("  各关节校准结果:\n");
        for (int i = 0; i < SO101_ARM_MOTOR_COUNT; i++) {
            printf("    %s: offset=%d range=[%u, %u]\n",
                            so101_joint_name(i),
                            calib.joints[i].homing_offset,
                            calib.joints[i].range_min,
                            calib.joints[i].range_max);
        }
    } else {
        printf("\n  ✗ 校准失败\n");
    }

    motor_free(motors, SO101_ARM_MOTOR_COUNT);

reinit:
    /* 重新初始化 */
    printf("\n  重新初始化机械臂...\n");
    g_arm = alloc_arm_on_current_port();
    if (g_arm)
        printf("  ✓ 机械臂就绪 (校准数据已自动加载)\n");
    else
        printf("  ✗ 机械臂初始化失败\n");

    printf("  重新初始化夹爪...\n");
    g_grip = alloc_grip_on_current_port();
    if (g_grip)
        printf("  ✓ 夹爪就绪\n");
    else
        printf("  ✗ 夹爪初始化失败\n");
}

/**
* 测试 11: 读取舵机寄存器 — 查看当前配置
*
* ⚠️ 会暂时释放机械臂/夹爪句柄。
*/
static void test_read_registers(void) {
    printf("\n=== 测试 11: 读取舵机寄存器 ===\n");

    /* 释放现有连接 */
    if (g_arm) {
        manip_stop(g_arm);
        manip_free(g_arm);
        g_arm = NULL;
    }
    if (g_grip) {
        grasp_execute(g_grip, GRASP_CMD_RELAX, 0.0f);
        grasp_free(g_grip);
        g_grip = NULL;
    }

    struct motor_dev *motors[SO101_ARM_MOTOR_COUNT];

    for (int i = 0; i < SO101_ARM_MOTOR_COUNT; i++) {
        motors[i] = motor_alloc_uart(
                "drv_uart_feetech", g_uart_path, 1000000,
                (uint8_t)(i + 1), NULL);
        if (!motors[i]) {
            fprintf(stderr, "  [错误] 分配电机 %d 失败\n", i + 1);
            motor_free(motors, (uint32_t)i);
            goto reinit;
        }
    }

    if (motor_init(motors, SO101_ARM_MOTOR_COUNT) != 0) {
        fprintf(stderr, "  [错误] 电机初始化失败\n");
        motor_free(motors, SO101_ARM_MOTOR_COUNT);
        goto reinit;
    }

    printf("  %-16s %5s %5s %5s %5s %5s %5s %5s %5s %5s %5s\n",
                    "关节", "Pos", "Mode", "P", "I", "D", "ACC", "OFS",
                    "MinAng", "MaxAng", "Temp");
    printf("  %-16s %5s %5s %5s %5s %5s %5s %5s %5s %5s %5s\n",
                    "----", "---", "----", "--", "--", "--", "---", "---",
                    "------", "------", "----");

    for (int i = 0; i < SO101_ARM_MOTOR_COUNT; i++) {
        int pos   = reg_read_word(motors[i], 56);  /* PRESENT_POSITION */
        int mode  = reg_read_byte(motors[i], 33);  /* MODE */
        int p_val = reg_read_byte(motors[i], STS3215_P_COEFFICIENT);
        int i_val = reg_read_byte(motors[i], STS3215_I_COEFFICIENT);
        int d_val = reg_read_byte(motors[i], STS3215_D_COEFFICIENT);
        int acc   = reg_read_byte(motors[i], 41);  /* ACC */
        int ofs_raw = reg_read_word(motors[i], 31);  /* OFS (signed) */
        int16_t ofs = (int16_t)ofs_raw;  /* Convert to signed */
        int min_a = reg_read_word(motors[i], 9);   /* MIN_ANGLE_LIMIT */
        int max_a = reg_read_word(motors[i], 11);  /* MAX_ANGLE_LIMIT */
        int temp  = reg_read_byte(motors[i], 63);  /* TEMPERATURE */

        printf("  %-16s %5d %5d %5d %5d %5d %5d %5d %5d %5d %5d°C\n",
                        so101_joint_name(i), pos, mode, p_val, i_val, d_val,
                        acc, ofs, min_a, max_a, temp);
    }

    motor_free(motors, SO101_ARM_MOTOR_COUNT);

reinit:
    /* 重新初始化 */
    g_arm = alloc_arm_on_current_port();
    g_grip = alloc_grip_on_current_port();
}

/**
* 测试 12: 正运动学 (FK) — 读取当前关节角，计算末端位姿
*/
static void test_fk(void) {
    printf("\n=== 测试 12: 正运动学 (FK) ===\n");

    if (!g_kin_solver) {
        printf("  [错误] 运动学求解器未初始化\n");
        printf("  请确保编译时启用了 TracIK (-DMANIP_ENABLE_TRACIK=ON)\n");
        return;
    }

    /* 读取当前关节角 */
    manip_joint_t joints;
    if (manip_get_state(g_arm, &joints, NULL) != MANIP_OK) {
        printf("  [错误] 读取关节角失败\n");
        return;
    }

    printf("  当前关节角:\n");
    print_joints(&joints);

    /* 转换为 kin_joints_t */
    kin_joints_t kin_joints;
    kin_joints.count = joints.count;
    for (int i = 0; i < joints.count && i < KIN_MAX_JOINTS; i++) {
        kin_joints.q[i] = (double)joints.joints[i];
    }

    /* 计算正运动学 */
    kin_pose_t kin_pose;
    int ret = kin_forward(g_kin_solver, &kin_joints, &kin_pose);
    if (ret != KIN_OK) {
        printf("  [错误] FK 计算失败 (err=%d)\n", ret);
        return;
    }

    /* 转换为 manip_pose_t 并显示 */
    manip_pose_t pose;
    pose.x = (float)kin_pose.x;
    pose.y = (float)kin_pose.y;
    pose.z = (float)kin_pose.z;
    pose.qw = (float)kin_pose.qw;
    pose.qx = (float)kin_pose.qx;
    pose.qy = (float)kin_pose.qy;
    pose.qz = (float)kin_pose.qz;

    printf("\n  ✓ FK 计算成功:\n");
    print_pose(&pose);
}

/**
* 测试 13: 逆运动学 (IK) — 输入目标位姿，计算关节角
*/
static void test_ik(void) {
    printf("\n=== 测试 13: 逆运动学 (IK) ===\n");

    if (!g_kin_solver) {
        printf("  [错误] 运动学求解器未初始化\n");
        return;
    }

    /* 读取当前关节角作为种子值 */
    manip_joint_t cur_joints;
    if (manip_get_state(g_arm, &cur_joints, NULL) != MANIP_OK) {
        printf("  [错误] 读取当前关节角失败\n");
        return;
    }

    printf("  当前关节角 (作为 IK 种子值):\n");
    print_joints(&cur_joints);

    /* 输入目标位姿 */
    printf("\n  输入目标位姿:\n");
    printf("    位置 x (m): ");
    float x = 0;
    if (scanf("%f", &x) != 1) {
        printf("  [错误] 输入无效\n");
        flush_stdin();
        return;
    }
    printf("    位置 y (m): ");
    float y = 0;
    if (scanf("%f", &y) != 1) {
        printf("  [错误] 输入无效\n");
        flush_stdin();
        return;
    }
    printf("    位置 z (m): ");
    float z = 0;
    if (scanf("%f", &z) != 1) {
        printf("  [错误] 输入无效\n");
        flush_stdin();
        return;
    }
    flush_stdin();

    /* 询问 IK 模式 */
    printf("\n  IK 模式:\n");
    printf("    1. 仅位置 IK (推荐，忽略姿态)\n");
    printf("    2. 位置+姿态 IK (保持当前姿态)\n");
    printf("  选择 (1/2): ");
    int mode = 1;
    if (scanf("%d", &mode) != 1) {
        printf("  [错误] 输入无效\n");
        flush_stdin();
        return;
    }
    flush_stdin();

    /* 姿态使用当前姿态 (简化输入) */
    if (mode == 2) {
        printf("  姿态: 使用当前姿态\n");
    } else {
        printf("  姿态: 忽略 (仅优化位置)\n");
    }

    /* 先计算当前姿态 */
    kin_joints_t seed;
    seed.count = cur_joints.count;
    for (int i = 0; i < cur_joints.count && i < KIN_MAX_JOINTS; i++) {
        seed.q[i] = (double)cur_joints.joints[i];
    }

    kin_pose_t cur_pose;
    if (kin_forward(g_kin_solver, &seed, &cur_pose) != KIN_OK) {
        printf("  [错误] 无法计算当前姿态\n");
        return;
    }

    /* 构造目标位姿 (位置用户输入，姿态保持当前) */
    kin_pose_t target;
    target.x = (double)x;
    target.y = (double)y;
    target.z = (double)z;
    target.qw = cur_pose.qw;
    target.qx = cur_pose.qx;
    target.qy = cur_pose.qy;
    target.qz = cur_pose.qz;

    printf("\n  目标位姿:\n");
    manip_pose_t target_manip;
    target_manip.x = (float)target.x;
    target_manip.y = (float)target.y;
    target_manip.z = (float)target.z;
    target_manip.qw = (float)target.qw;
    target_manip.qx = (float)target.qx;
    target_manip.qy = (float)target.qy;
    target_manip.qz = (float)target.qz;
    print_pose(&target_manip);

    /* 计算逆运动学 */
    kin_ik_params_t ik_params = {0};
    if (mode == 1) {
        /* 仅位置 IK: position_weight = 1.0 */
        ik_params.position_weight = 1.0;
        ik_params.epsilon = 1e-3;  /* 1mm 精度 */
    } else {
        /* 均衡 IK: position_weight = 0.5 */
        ik_params.position_weight = 0.5;
        ik_params.epsilon = 5e-4;  /* 0.5mm 精度 */
    }

    kin_joints_t result;
    int ret = kin_inverse(g_kin_solver, &target, &seed, &ik_params, &result);
    if (ret != KIN_OK) {
        printf("\n  [错误] IK 求解失败 (err=%d)\n", ret);
        printf("  可能原因: 目标位姿超出工作空间或奇异位形\n");
        return;
    }

    /* 显示结果 */
    printf("\n  ✓ IK 求解成功:\n");
    manip_joint_t result_joints;
    result_joints.count = result.count;
    for (int i = 0; i < result.count && i < MANIP_MAX_JOINTS; i++) {
        result_joints.joints[i] = (float)result.q[i];
    }
    print_joints(&result_joints);

    /* 询问是否执行 */
    printf("\n  是否执行该关节角？(y/n): ");
    char c = 0;
    if (scanf("%c", &c) != 1 || (c != 'y' && c != 'Y')) {
        printf("  已取消\n");
        flush_stdin();
        return;
    }
    flush_stdin();

    printf("  执行运动...\n");
    ret = manip_move_joints(g_arm, &result_joints, 0.3f);
    if (ret != MANIP_OK) {
        printf("  [错误] 运动失败 (err=%d)\n", ret);
        return;
    }

    for (int i = 0; i < 30 && g_running; i++) {
        usleep(100000);
        manip_tick(g_arm, 0.1f);
    }

    printf("  运动完成\n");
}

/**
* 测试 14: FK/IK 往返测试 — 验证一致性
*/
static void test_fk_ik_roundtrip(void) {
    printf("\n=== 测试 14: FK/IK 往返测试 ===\n");

    if (!g_kin_solver) {
        printf("  [错误] 运动学求解器未初始化\n");
        return;
    }

    /* 读取当前关节角 */
    manip_joint_t joints;
    if (manip_get_state(g_arm, &joints, NULL) != MANIP_OK) {
        printf("  [错误] 读取关节角失败\n");
        return;
    }

    printf("  [1/4] 当前关节角:\n");
    print_joints(&joints);

    /* 转换为 kin_joints_t */
    kin_joints_t q_orig;
    q_orig.count = joints.count;
    for (int i = 0; i < joints.count && i < KIN_MAX_JOINTS; i++) {
        q_orig.q[i] = (double)joints.joints[i];
    }

    /* FK: 关节角 → 位姿 */
    kin_pose_t pose;
    int ret = kin_forward(g_kin_solver, &q_orig, &pose);
    if (ret != KIN_OK) {
        printf("  [错误] FK 失败 (err=%d)\n", ret);
        return;
    }

    printf("\n  [2/4] FK 计算的末端位姿:\n");
    manip_pose_t pose_manip;
    pose_manip.x = (float)pose.x;
    pose_manip.y = (float)pose.y;
    pose_manip.z = (float)pose.z;
    pose_manip.qw = (float)pose.qw;
    pose_manip.qx = (float)pose.qx;
    pose_manip.qy = (float)pose.qy;
    pose_manip.qz = (float)pose.qz;
    print_pose(&pose_manip);

    /* IK: 位姿 → 关节角 (用原始关节角作为种子) */
    kin_joints_t q_result;
    ret = kin_inverse(g_kin_solver, &pose, &q_orig, NULL, &q_result);
    if (ret != KIN_OK) {
        printf("\n  [错误] IK 失败 (err=%d)\n", ret);
        return;
    }

    printf("\n  [3/4] IK 计算的关节角:\n");
    manip_joint_t result_joints;
    result_joints.count = q_result.count;
    for (int i = 0; i < q_result.count && i < MANIP_MAX_JOINTS; i++) {
        result_joints.joints[i] = (float)q_result.q[i];
    }
    print_joints(&result_joints);

    /* 计算误差 */
    printf("\n  [4/4] 往返误差分析:\n");
    double max_error = 0;
    for (int i = 0; i < q_orig.count && i < q_result.count; i++) {
        double error = fabs(q_result.q[i] - q_orig.q[i]);
        if (error > max_error)
            max_error = error;
        printf("    关节 %d: %.4f rad (%.2f°)\n", i + 1, error,
                        rad2deg((float)error));
    }

    printf("\n  最大误差: %.4f rad (%.2f°)\n",
                max_error, rad2deg((float)max_error));
    if (max_error < 0.01) {
        printf("  ✓ 往返一致性良好 (误差 < 0.01 rad)\n");
    } else if (max_error < 0.05) {
        printf("  ⚠ 往返误差较大 (0.01~0.05 rad)\n");
    } else {
        printf("  ✗ 往返误差过大 (> 0.05 rad)\n");
    }
}

/**
* 测试 15: 直线运动 (move_line) — TCP 空间笛卡尔插补
*
* 流程：
*   1. FK 获取当前 TCP 位姿
*   2. 用户输入目标位置（或使用默认偏移 +5cm Z）
*   3. 调用 manip_move_line
*   4. tick 循环等待完成
*   5. FK 验证实际到达位置误差
*/
static void test_move_line(void) {
    printf("\n=== 测试 15: 直线运动 (move_line) ===\n");

    if (!g_kin_solver) {
        printf("  [错误] 运动学求解器未初始化\n");
        return;
    }

    /* 1. 读取当前关节角 + FK 计算当前 TCP */
    manip_joint_t joints;
    if (manip_get_state(g_arm, &joints, NULL) != MANIP_OK) {
        printf("  [错误] 读取关节角失败\n");
        return;
    }

    kin_joints_t q_cur;
    q_cur.count = joints.count;
    for (int i = 0; i < joints.count; i++)
        q_cur.q[i] = (double)joints.joints[i];

    kin_pose_t fk_now;
    if (kin_forward(g_kin_solver,
                                    &q_cur, &fk_now) != KIN_OK) {
        printf("  [错误] FK 计算失败\n");
        return;
    }

    printf("  当前 TCP: x=%.4f y=%.4f z=%.4f\n",
                    fk_now.x, fk_now.y, fk_now.z);

    /* 2. 获取目标位置 */
    manip_pose_t target = {0};
    printf("\n  输入目标方式:\n");
    printf("    1. 手动输入 x y z (m)\n");
    printf("    2. 默认: Z +0.05m\n");
    printf("  选择 [1/2]: ");

    int mode = 2;
    if (scanf("%d", &mode) != 1)
        mode = 2;
    flush_stdin();

    if (mode == 1) {
        printf("  输入 x y z (m): ");
        if (scanf("%f %f %f",
                            &target.x, &target.y, &target.z) != 3) {
            printf("  [错误] 输入格式错误\n");
            flush_stdin();
            return;
        }
        flush_stdin();
    } else {
        target.x = (float)fk_now.x;
        target.y = (float)fk_now.y;
        target.z = (float)fk_now.z + 0.05f;
    }
    /* 姿态保持当前 */
    target.qw = (float)fk_now.qw;
    target.qx = (float)fk_now.qx;
    target.qy = (float)fk_now.qy;
    target.qz = (float)fk_now.qz;

    printf("\n  目标 TCP: x=%.4f y=%.4f z=%.4f\n",
                    target.x, target.y, target.z);
    double dx = target.x - fk_now.x;
    double dy = target.y - fk_now.y;
    double dz = target.z - fk_now.z;
    printf("  距离: %.4f m\n",
                    sqrt(dx * dx + dy * dy + dz * dz));

    /* 3. 调用 move_line */
    printf("\n  执行 move_line...\n");
    int ret = manip_move_line(g_arm, &target, 0.5f);
    if (ret != MANIP_OK) {
        printf("  [错误] move_line 失败 (ret=%d)\n", ret);
        return;
    }

    /* 4. tick 循环等待轨迹完成 + 舵机到位 */
    /*
    * 每次 tick 推进一个 waypoint (最多 64 步)，
    * 轨迹完成后再多给 settle_extra 个 tick
    * 让舵机到达最后一个 waypoint 的目标位置。
    * 总耗时 ≈ (steps + settle) × 50ms
    */
    int settle_extra = 40;  /* 2s settling @20Hz */
    int max_ticks = 64 + settle_extra; /* 64 = max waypoints */
    if (max_ticks > 200)
        max_ticks = 200;

    printf("  轨迹执行中");
    for (int t = 0; t < max_ticks; t++) {
        manip_tick(g_arm, 0.05f);
        usleep(50000); /* 50ms = 20Hz */

        if (t % 10 == 0) {
            printf(".");
            fflush(stdout);
        }
    }
    printf(" 完成\n");

    /* 5. FK 验证实际到达位置 */
    manip_joint_t final_j;
    manip_get_state(g_arm, &final_j, NULL);

    kin_joints_t q_final;
    q_final.count = final_j.count;
    for (int i = 0; i < final_j.count; i++)
        q_final.q[i] = (double)final_j.joints[i];

    kin_pose_t fk_final;
    if (kin_forward(g_kin_solver,
                                    &q_final, &fk_final) == KIN_OK) {
        printf("\n  实际 TCP: x=%.4f y=%.4f z=%.4f\n",
                        fk_final.x, fk_final.y, fk_final.z);
        double ex = target.x - fk_final.x;
        double ey = target.y - fk_final.y;
        double ez = target.z - fk_final.z;
        double err = sqrt(ex * ex + ey * ey + ez * ez);
        printf("  位置误差: %.4f m (%.2f mm)\n",
                        err, err * 1000.0);
        if (err < 0.005)
            printf("  ✓ 精度良好 (<5mm)\n");
        else if (err < 0.02)
            printf("  ⚠ 精度一般 (<20mm)\n");
        else
            printf("  ✗ 精度较差 (>20mm)\n");
    }
}

/* ========================================================================== */
/* 菜单 & 主函数 */
/* ========================================================================== */

static void print_menu(void) {
    printf("\n");
    printf("╔═══════════════════════════════════════╗\n");
    printf("║   SO-101 机械臂硬件测试               ║\n");
    printf("╠═══════════════════════════════════════╣\n");
    printf("║  1. 读取关节状态                       ║\n");
    printf("║  2. 单轴运动                           ║\n");
    printf("║  3. 关节归零 (所有轴回 0°)             ║\n");
    printf("║  4. 挥手演示动作                       ║\n");
    printf("║  5. 末端执行器控制                     ║\n");
    printf("║  6. 急停 (释放所有扭矩)               ║\n");
    printf("║  7. 状态监控 (连续打印)               ║\n");
    printf("║  8. 手动输入 5 轴角度                  ║\n");
    printf("║ ─── 组装 & 校准 ────────────────────  ║\n");
    printf("║  9. 组装 (Assemble) — 配置舵机参数     ║\n");
    printf("║ 10. 校准 (Calibrate) — 中位 + 行程     ║\n");
    printf("║ 11. 读取舵机寄存器                     ║\n");
    printf("║ ─── 运动学测试 ──────────────────────  ║\n");
    printf("║ 12. 正运动学 (FK) 测试                 ║\n");
    printf("║ 13. 逆运动学 (IK) 测试                 ║\n");
    printf("║ 14. FK/IK 往返一致性测试               ║\n");
    printf("║ 15. 直线运动 (move_line) 测试          ║\n");
    printf("║  0. 退出                               ║\n");
    printf("╚═══════════════════════════════════════╝\n");
    printf("  请选择: ");
}

int main(int argc, char *argv[]) {
    /* 解析命令行参数 */
    if (argc > 1) {
        g_uart_path = argv[1];
    }

    printf("============================================\n");
    printf("  SO-101 机械臂硬件测试程序\n");
    printf("  串口: %s\n", g_uart_path);
    printf("  波特率: 1000000\n");
    printf("  关节 ID: 1-5, 夹爪 ID: 6\n");
    printf("============================================\n\n");

    /* 注册信号处理 */
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    /* -----------------------------------------------------------
    * 初始化机械臂 (如果需要自定义串口路径，通过 config 传入)
    * ----------------------------------------------------------- */
    printf("[1/2] 初始化机械臂 (5 DOF)...\n");

    g_arm = alloc_arm_on_current_port();
    if (!g_arm) {
        fprintf(stderr, "[错误] 机械臂初始化失败!\n");
        fprintf(stderr, "  可能原因:\n");
        fprintf(stderr, "  - 串口 %s 不存在 "
                        "(检查 ls /dev/ttyACM*)\n", g_uart_path);
        fprintf(stderr, "  - 权限不足 (用 sudo 运行或加入 dialout 组)\n");
        fprintf(stderr, "  - 舵机未上电或接线错误\n");
        fprintf(stderr, "  - 波特率不匹配 (默认 1Mbaud)\n");
        return 1;
    }
    printf("  ✓ 机械臂初始化成功\n");

    /* -----------------------------------------------------------
    * 初始化夹爪
    * ----------------------------------------------------------- */
    printf("[2/2] 初始化夹爪 (ID=6)...\n");
    g_grip = alloc_grip_on_current_port();
    if (!g_grip) {
        fprintf(stderr, "[错误] 夹爪初始化失败!\n");
        return 1;
    }
    printf("  ✓ 夹爪初始化成功\n");

    /* -----------------------------------------------------------
    * 初始化运动学求解器 (可选)
    * ----------------------------------------------------------- */
    printf("[3/3] 初始化运动学求解器...\n");
    g_kin_solver = kin_create(NULL,
                                SO101_DEFAULT_URDF_PATH,
                                SO101_BASE_LINK,
                                SO101_TIP_LINK);
    if (g_kin_solver) {
        printf("  ✓ 运动学求解器初始化成功\n");
        printf("  (FK/IK 测试功能已启用)\n");
        /* 绑定到机械臂 */
        manip_set_kinematics(g_arm, g_kin_solver);
    } else {
        printf("  ⚠ 运动学求解器初始化失败\n");
        printf("  (可能未编译 TracIK，FK/IK 测试将不可用)\n");
    }

    /* 首次读取状态 */
    printf("\n[初始状态]\n");
    test_read_state();

    /* -----------------------------------------------------------
    * 交互菜单循环
    * ----------------------------------------------------------- */
    while (g_running) {
        print_menu();

        int choice = -1;
        if (scanf("%d", &choice) != 1) {
            flush_stdin();
            continue;
        }
        flush_stdin();

        switch (choice) {
        case 1:
            test_read_state();
            break;
        case 2:
            test_single_joint();
            break;
        case 3:
            test_home();
            break;
        case 4:
            test_wave();
            break;
        case 5:
            if (g_grip)
                test_gripper();
            else
                printf("  [!] 夹爪未初始化\n");
            break;
        case 6:
            test_stop();
            break;
        case 7:
            test_monitor();
            break;
        case 8:
            test_manual_move();
            break;
        case 9:
            test_assemble();
            break;
        case 10:
            test_calibrate();
            break;
        case 11:
            test_read_registers();
            break;
        case 12:
            test_fk();
            break;
        case 13:
            test_ik();
            break;
        case 14:
            test_fk_ik_roundtrip();
            break;
        case 15:
            test_move_line();
            break;
        case 0:
            g_running = 0;
            break;
        default:
            printf("  无效选择\n");
            break;
        }
    }

    /* -----------------------------------------------------------
    * 清理
    * ----------------------------------------------------------- */
    printf("\n[清理] 停止并释放资源...\n");
    if (g_arm) {
        manip_stop(g_arm);
        manip_free(g_arm);
    }
    if (g_grip) {
        grasp_execute(g_grip, GRASP_CMD_RELAX, 0.0f);
        grasp_free(g_grip);
    }
    if (g_kin_solver && !g_arm) {
        /* 仅在机械臂未初始化时手动释放求解器。
        * 正常情况下 manip_set_kinematics 已将所有权转移给 g_arm，
        * manip_free(g_arm) 会自动释放, 这里再释放会导致 double-free 崩溃。 */
        kin_destroy(g_kin_solver);
    }
    printf("再见!\n");
    return 0;
}
