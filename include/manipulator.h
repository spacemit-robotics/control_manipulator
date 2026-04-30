/*
* Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
* SPDX-License-Identifier: Apache-2.0
*
* @file manipulator.h
* @brief 通用机械臂控制接口 (公共 API)
*
* 本头文件定义了硬件无关的机械臂控制 API。
* 具体驱动 (如 SO-101 Feetech) 通过注册机制挂载。
*/

#ifndef MANIPULATOR_H
#define MANIPULATOR_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
* 1. Constants & Error Codes
* ========================================================================== */

#define MANIP_MAX_JOINTS 7

#define MANIP_OK           0
#define MANIP_ERR_ALLOC   -1
#define MANIP_ERR_CONNECT -2
#define MANIP_ERR_TIMEOUT -3
#define MANIP_ERR_CONFIG  -4
#define MANIP_ERR_PARAM   -5
#define MANIP_ERR_NOSYS   -6  /* 功能未实现 */

/* ==========================================================================
* 2. Data Structures
* ========================================================================== */

/** 关节空间 */
typedef struct {
    uint8_t count;
    float joints[MANIP_MAX_JOINTS]; /* 单位: rad */
} manip_joint_t;

/** 笛卡尔空间 (末端位姿) */
typedef struct {
    float x, y, z;         /* 单位: m */
    float qw, qx, qy, qz; /* 姿态四元数 */
} manip_pose_t;

/** 状态枚举 */
typedef enum {
    MANIP_IDLE = 0,
    MANIP_MOVING,
    MANIP_TEACHING, /* 示教模式 */
    MANIP_ERROR
} manip_state_t;

/* ==========================================================================
* 3. Opaque Handle
* ========================================================================== */

struct manip_dev;

/* ==========================================================================
* 4. API Functions
* ========================================================================== */

/**
* @brief 创建机械臂实例
* @param driver_name 驱动名 (如 "so101", "dummy")
* @param args        驱动特定配置参数 (透传给驱动 factory)
* @return 成功返回设备句柄，失败返回 NULL
*/
struct manip_dev *manip_alloc(const char *driver_name, void *args);

/**
* @brief 释放机械臂实例
*/
void manip_free(struct manip_dev *dev);

/* --- 运动控制 --- */

/**
* @brief 关节运动 (PTP / MoveJ)
* @param target      目标关节角度
* @param speed_ratio 速度倍率 [0.0 ~ 1.0]
*/
int manip_move_joints(struct manip_dev *dev,
    const manip_joint_t *target,
    float speed_ratio);

/**
* @brief 直线运动 (Linear / MoveL)
* @note  需要底层驱动支持逆解 (IK)，未实现的驱动返回 MANIP_ERR_NOSYS
*/
int manip_move_line(struct manip_dev *dev,
    const manip_pose_t *target,
    float speed_ratio);

/**
* @brief 目标位姿运动 (用于遥操作)
* @note  直接IK转关节角度，无笛卡尔插补，适合高频调用
*/
int manip_move_target(struct manip_dev *dev,
    const manip_pose_t *target,
    float speed_ratio);

/**
* @brief 求解目标位姿对应的关节角, 但不下发运动命令
* @note  适合上层在 IK 结果基础上做局部 joint override 后再一次性 move_joints
*/
int manip_solve_target_joints(struct manip_dev *dev,
    const manip_pose_t *target,
    manip_joint_t *out_joints);

/**
* @brief 停止/急停
*/
void manip_stop(struct manip_dev *dev);

/**
* @brief 设置工具中心点 (TCP) 偏移
* @note  未实现的驱动返回 MANIP_ERR_NOSYS
*/
int manip_set_tcp(struct manip_dev *dev, const manip_pose_t *tcp_offset);

/* --- 模式设置 --- */

/**
* @brief 开启/关闭示教模式 (拖拽)
* @note  未实现的驱动返回 MANIP_ERR_NOSYS
*/
int manip_set_teach_mode(struct manip_dev *dev, bool enable);

/* --- 反馈查询 --- */

/**
* @brief 获取实时状态
* @param out_joints 当前关节角 (Encoder)，可为 NULL
* @param out_pose   当前末端位姿 (FK)，可为 NULL
*/
int manip_get_state(struct manip_dev *dev,
    manip_joint_t *out_joints,
    manip_pose_t *out_pose);

/**
* @brief 周期计算 (建议高频调用, e.g. 200Hz)
* @param dt_s 时间间隔 (秒)
*/
void manip_tick(struct manip_dev *dev, float dt_s);

/* --- 运动学 --- */

struct kin_solver;  /* 前向声明，避免依赖 kinematics_interface.h */

/**
* @brief 绑定运动学求解器到机械臂实例
*
* 绑定后，manip_move_line() 会自动使用 IK → move_joints 实现直线运动，
* manip_get_state() 会自动使用 FK 计算末端位姿。
*
* @param dev    机械臂句柄
* @param solver 运动学求解器 (由 kin_create() 创建)，可为 NULL 解绑
* @return MANIP_OK 或错误码
* @note  所有权转移给 dev，manip_free() 会自动调用 kin_destroy() 释放。
*        重复绑定新 solver 时，旧 solver 会被自动销毁。
*/
int manip_set_kinematics(struct manip_dev *dev, struct kin_solver *solver);

#ifdef __cplusplus
}
#endif

#endif  // MANIPULATOR_H
