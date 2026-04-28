/*
* Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
* SPDX-License-Identifier: Apache-2.0
*
* @file kinematics_interface.h
* @brief 运动学求解器抽象接口 (FK / IK)
*
* 设计思路：
*   - 纯 C 接口，可由 C++ 后端实现 (TracIK / KDL / Pinocchio 等)
*   - 使用 opaque handle + 函数指针表，运行时可切换求解器
*   - components 层可选编译；middleware/ros2 层可直接调用或使用 ROS2 原生接口
*
* 使用方式：
*   1. 调用 kin_create() 创建求解器实例 (需要 URDF 路径 + 链起止关节名)
*   2. 调用 kin_forward() / kin_inverse() 进行正/逆运动学计算
*   3. 调用 kin_destroy() 释放资源
*
* 扩展新求解器：
*   1. 实现 struct kin_ops 中的函数
*   2. 在 kin_create 工厂中注册
*/
#ifndef MANIPULATOR_INCLUDE_KINEMATICS_INTERFACE_H_
#define MANIPULATOR_INCLUDE_KINEMATICS_INTERFACE_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
* 1. Error Codes
* ========================================================================== */

#define KIN_OK              0   /* 成功 */
#define KIN_ERR_ALLOC      -1   /* 内存分配失败 */
#define KIN_ERR_PARAM      -2   /* 参数错误 */
#define KIN_ERR_URDF       -3   /* URDF 解析失败 */
#define KIN_ERR_NOSOLVER   -4   /* 求解器不存在 / 未编译 */
#define KIN_ERR_IK_FAIL    -5   /* 逆解求解失败 (无解或未收敛) */
#define KIN_ERR_FK_FAIL    -6   /* 正解计算失败 */
#define KIN_ERR_NOSYS      -7   /* 功能未实现 */

/* ==========================================================================
* 2. Constants
* ========================================================================== */

#define KIN_MAX_JOINTS  12  /* 单链最大关节数 */

/* ==========================================================================
* 3. Data Structures
* ========================================================================== */

/**
* @brief 关节角度组
*/
typedef struct {
    uint8_t count;                    /* 关节数量 */
    double  q[KIN_MAX_JOINTS];       /* 关节角度 (rad) */
} kin_joints_t;

/**
* @brief 笛卡尔空间位姿 (位置 + 四元数)
*/
typedef struct {
    double x, y, z;          /* 位置 (m) */
    double qw, qx, qy, qz;  /* 姿态四元数 (w, x, y, z) */
} kin_pose_t;

/**
* @brief IK 求解参数 (可选)
*/
typedef struct {
    double  timeout_s;        /* 超时 (秒), 0 使用默认值 */
    double  epsilon;          /* 收敛精度, 0 使用默认值 */
    double  position_weight;  /* 位置权重 [0~1]: 1.0=仅位置, 0.9=位置强+姿态弱, 0.5=均衡, 0 使用默认 1.0 */
} kin_ik_params_t;

/* ==========================================================================
* 4. Opaque Handle
* ========================================================================== */

/** 求解器实例 (前向声明) */
typedef struct kin_solver kin_solver_t;

/* ==========================================================================
* 5. Solver Operations Table (供求解器后端实现)
* ========================================================================== */

/**
* @brief 求解器虚函数表
*
* 新的求解器后端只需填充这个结构体。
*/
struct kin_ops {
    /**
    * @brief 正运动学: 关节角 → 末端位姿
    * @param solver  求解器实例
    * @param joints  输入关节角 (rad)
    * @param out     输出末端位姿
    * @return KIN_OK 或错误码
    */
    int (*forward)(kin_solver_t *solver,
        const kin_joints_t *joints,
        kin_pose_t *out);

    /**
    * @brief 逆运动学: 末端位姿 → 关节角
    * @param solver  求解器实例
    * @param target  目标末端位姿
    * @param q_init  初始猜测 (种子值), 可为 NULL
    * @param params  求解参数, 可为 NULL (使用默认)
    * @param out     输出关节角
    * @return KIN_OK 或 KIN_ERR_IK_FAIL
    */
    int (*inverse)(kin_solver_t *solver,
        const kin_pose_t *target,
        const kin_joints_t *q_init,
        const kin_ik_params_t *params,
        kin_joints_t *out);

    /**
    * @brief 获取关节数量
    * @return 链中的关节数
    */
    int (*get_num_joints)(kin_solver_t *solver);

    /**
    * @brief 获取关节限位
    * @param lower  输出下限 (rad), 大小至少为 get_num_joints()
    * @param upper  输出上限 (rad)
    * @return KIN_OK 或错误码
    */
    int (*get_joint_limits)(kin_solver_t *solver,
        double *lower, double *upper);

    /**
    * @brief 释放求解器资源
    */
    void (*destroy)(kin_solver_t *solver);
};

/* ==========================================================================
* 6. Solver Instance Structure
* ========================================================================== */

/**
* @brief 求解器实例
*
* 后端在 create 时分配此结构，将 ops 指向自己的实现，
* priv_data 存放后端私有数据。
*/
struct kin_solver {
    const char          *name;       /* 求解器名称 (如 "tracik", "dummy") */
    const struct kin_ops *ops;       /* 虚函数表 */
    void                *priv_data;  /* 后端私有数据 */
};

/* ==========================================================================
* 7. Solver Registry (类似 manipulator 的驱动注册模式)
* ========================================================================== */

/** 求解器工厂函数原型 */
typedef kin_solver_t *(*kin_factory_t)(const char *urdf_path,
    const char *base_link,
    const char *tip_link);

/** 求解器注册信息 */
struct kin_solver_info {
    const char          *name;     /* 求解器名 (如 "tracik", "kdl") */
    kin_factory_t        factory;  /* 工厂函数 */
    struct kin_solver_info *next;  /* 链表 */
};

/**
* @brief 注册一个求解器后端 (由 constructor 自动调用)
*/
void kin_solver_register(struct kin_solver_info *info);

/**
* @brief 注册宏 (与 REGISTER_MANIP_DRIVER 风格一致)
*/
#define REGISTER_KIN_SOLVER(_name, _factory)                                   \
    static struct kin_solver_info __kin_info_##_factory = {                       \
            .name = _name, .factory = _factory, .next = NULL};                       \
    __attribute__((constructor)) static void __auto_reg_kin_##_factory(void) {    \
        kin_solver_register(&__kin_info_##_factory);                                \
    }

/* ==========================================================================
* 8. Public API
* ========================================================================== */

/**
* @brief 创建运动学求解器
*
* @param solver_name 求解器名称 (如 "tracik", "kdl", "dummy")
*                    传 NULL 或 "" 时使用第一个已注册的求解器
* @param urdf_path   URDF 文件路径
* @param base_link   运动链基座关节名 (如 "base_link")
* @param tip_link    运动链末端关节名 (如 "tool0")
* @return 成功返回求解器句柄，失败返回 NULL
*/
kin_solver_t *kin_create(const char *solver_name,
    const char *urdf_path,
    const char *base_link,
    const char *tip_link);

/**
* @brief 销毁求解器，释放所有资源
*/
void kin_destroy(kin_solver_t *solver);

/**
* @brief 正运动学
*/
int kin_forward(kin_solver_t *solver,
    const kin_joints_t *joints,
    kin_pose_t *out);

/**
* @brief 逆运动学
*/
int kin_inverse(kin_solver_t *solver,
    const kin_pose_t *target,
    const kin_joints_t *q_init,
    const kin_ik_params_t *params,
    kin_joints_t *out);

/**
* @brief 获取关节数量
*/
int kin_get_num_joints(kin_solver_t *solver);

/**
* @brief 获取关节限位
*/
int kin_get_joint_limits(kin_solver_t *solver,
    double *lower, double *upper);

#ifdef __cplusplus
}
#endif

#endif  // MANIPULATOR_INCLUDE_KINEMATICS_INTERFACE_H_
