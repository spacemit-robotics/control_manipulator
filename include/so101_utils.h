/*
* Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
* SPDX-License-Identifier: Apache-2.0
* SO-101 机械臂工具函数 (组装 / 校准)
*
* 这些函数直接操作 motor_dev 层，用于:
*   - 首次组装时配置舵机参数
*   - 校准中位偏移和行程范围
*   - 测试程序中的交互式组装/校准流程
*
* 使用前需要先通过 motor_alloc_uart + motor_init 创建并初始化电机。
*/
#ifndef MANIPULATOR_INCLUDE_SO101_UTILS_H_
#define MANIPULATOR_INCLUDE_SO101_UTILS_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

struct motor_dev; /* 前置声明 */

#define SO101_ARM_MOTOR_COUNT 5
#define SO101_DEFAULT_CALIBRATION_PATH "./config/so101_calibration.json"
#define SO101_DEFAULT_URDF_PATH        "./urdf/so101.urdf"

/* SO-101 运动链定义 (与 URDF 中的 link name 对应) */
#define SO101_BASE_LINK   "base_link"
#define SO101_TIP_LINK    "gripper_frame_link"  /* 末端 TCP (含固定偏移) */
#define SO101_GRIP_LINK   "gripper_link"        /* 夹爪基座 (不含 TCP 偏移) */

/**
* @brief 组装过程 — 安全地初始化 SO-101 舵机参数
*
* 流程:
*   1. 关闭所有舵机扭矩
*   2. 配置舵机参数 (PID=16/0/0, ACC=254, Return_Delay=0)
*   3. 验证参数
*
* @param motors  5 个关节电机句柄数组
* @param count   关节数量 (必须为 5)
* @return 0 成功，-1 失败
*/
int so101_assemble(struct motor_dev **motors, int count);

/**
* @brief 单个关节的校准数据
*/
struct so101_joint_calib {
    int16_t homing_offset;
    uint16_t range_min;
    uint16_t range_max;
};

/**
* @brief SO-101 校准数据
*/
struct so101_calibration {
    bool valid;
    struct so101_joint_calib joints[SO101_ARM_MOTOR_COUNT];
};

/**
* @brief 完整校准过程
*
* @param motors      5 个关节电机句柄数组
* @param calib       校准数据 (输出)
* @param interactive 是否交互模式 (等待用户移动关节)
* @param save_path   校准文件保存路径，NULL 使用默认路径
* @return 0 成功，-1 失败
*/
int so101_calibrate(struct motor_dev **motors,
    struct so101_calibration *calib,
    bool interactive,
    const char *save_path);

/**
* @brief 获取关节名称
*
* @param index 关节索引 (0-4)
* @return 关节名称字符串，越界返回 "unknown"
*/
const char *so101_joint_name(int index);

#ifdef __cplusplus
}
#endif

#endif  // MANIPULATOR_INCLUDE_SO101_UTILS_H_
