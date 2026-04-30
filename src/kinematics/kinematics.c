/*
* Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
* SPDX-License-Identifier: Apache-2.0
*
* @file kinematics.c
* @brief Kinematics solver framework — 注册、创建、销毁及 API 分发
*
* 与 manipulator.c 的驱动注册模式一致：
*   - 各后端通过 REGISTER_KIN_SOLVER 宏 + GCC constructor 自动注册
*   - kin_create() 按名称查找后端并调用其工厂函数
*   - kin_forward/kin_inverse 通过 ops 分发
*/

#include "../include/kinematics_interface.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ==========================================================================
* Solver Registry (linked list)
* ========================================================================== */

static struct kin_solver_info *g_kin_list = NULL;

void kin_solver_register(struct kin_solver_info *info) {
    if (!info)
            return;
    info->next = g_kin_list;
    g_kin_list = info;
    printf("[KIN] Registered solver: %s\n", info->name);
}

static struct kin_solver_info *find_kin_solver(const char *name) {
    struct kin_solver_info *curr = g_kin_list;

    /* 如果 name 为空或 ""，优先查找 pinocchio，再 fallback 到链表头 */
    if (!name || name[0] == '\0') {
        struct kin_solver_info *it = g_kin_list;
        while (it) {
            if (it->name && strcmp(it->name, "pinocchio") == 0) {
                printf("[KIN] Using default solver: %s\n", it->name);
                return it;
            }
            it = it->next;
        }
        /* pinocchio 未注册，fallback 到第一个可用后端 */
        if (curr) {
            printf("[KIN] Using fallback solver: %s\n", curr->name);
            return curr;
        }
        printf("[KIN] No solver registered\n");
        return NULL;
    }

    while (curr) {
        if (curr->name && strcmp(curr->name, name) == 0)
                return curr;
        curr = curr->next;
    }

    printf("[KIN] Solver not found: %s\n", name);
    return NULL;
}

/* ==========================================================================
* Public API
* ========================================================================== */

kin_solver_t *kin_create(const char *solver_name,
                            const char *urdf_path,
                            const char *base_link,
                            const char *tip_link) {
    struct kin_solver_info *info = find_kin_solver(solver_name);
    if (!info || !info->factory) {
        printf("[KIN] No solver factory for: %s\n",
                        solver_name ? solver_name : "(null)");
        return NULL;
    }

    return info->factory(urdf_path, base_link, tip_link);
}

void kin_destroy(kin_solver_t *solver) {
    if (!solver)
            return;

    if (solver->ops && solver->ops->destroy) {
            solver->ops->destroy(solver);
            return;
    }

    /* 兜底释放 */
    free(solver->priv_data);
    free((void *)solver->name);
    free(solver);
}

int kin_forward(kin_solver_t *solver,
                const kin_joints_t *joints,
                kin_pose_t *out) {
    if (!solver || !joints || !out)
        return KIN_ERR_PARAM;

    if (solver->ops && solver->ops->forward)
        return solver->ops->forward(solver, joints, out);

    return KIN_ERR_NOSYS;
}

int kin_inverse(kin_solver_t *solver,
                const kin_pose_t *target,
                const kin_joints_t *q_init,
                const kin_ik_params_t *params,
                kin_joints_t *out) {
    if (!solver || !target || !out)
        return KIN_ERR_PARAM;

    if (solver->ops && solver->ops->inverse)
        return solver->ops->inverse(solver, target, q_init, params, out);

    return KIN_ERR_NOSYS;
}

int kin_get_num_joints(kin_solver_t *solver) {
    if (!solver)
        return KIN_ERR_PARAM;

    if (solver->ops && solver->ops->get_num_joints)
        return solver->ops->get_num_joints(solver);

    return KIN_ERR_NOSYS;
}

int kin_get_joint_limits(kin_solver_t *solver,
                            double *lower, double *upper) {
    if (!solver)
        return KIN_ERR_PARAM;

    if (solver->ops && solver->ops->get_joint_limits)
        return solver->ops->get_joint_limits(solver, lower, upper);

    return KIN_ERR_NOSYS;
}
