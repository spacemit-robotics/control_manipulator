/*
* Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
* SPDX-License-Identifier: Apache-2.0
*
* @file kinematics_dummy.c
* @brief Dummy kinematics solver (stub implementation)
*
* 当没有编译真实的 IK 库时，使用此 dummy 实现。
* 所有函数返回 KIN_ERR_NOSYS，表示功能未实现。
*/

#include "../include/kinematics_interface.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ==========================================================================
* Dummy Solver Private Data
* ========================================================================== */

typedef struct {
    char *urdf_path;
    char *base_link;
    char *tip_link;
    int   num_joints;  /* 默认 5 (SO-101) */
} dummy_priv_t;

/* ==========================================================================
* Dummy Operations
* ========================================================================== */

static int dummy_forward(kin_solver_t *solver,
                            const kin_joints_t *joints,
                            kin_pose_t *out) {
    (void)solver;
    (void)joints;
    (void)out;
    return KIN_ERR_NOSYS;
}

static int dummy_inverse(kin_solver_t *solver,
                            const kin_pose_t *target,
                            const kin_joints_t *q_init,
                            const kin_ik_params_t *params,
                            kin_joints_t *out) {
    (void)solver;
    (void)target;
    (void)q_init;
    (void)params;
    (void)out;
    return KIN_ERR_NOSYS;
}

static int dummy_get_num_joints(kin_solver_t *solver) {
    dummy_priv_t *priv = (dummy_priv_t *)solver->priv_data;
    return priv ? priv->num_joints : 5;
}

static int dummy_get_joint_limits(kin_solver_t *solver,
                                    double *lower,
                                    double *upper) {
    (void)solver;
    (void)lower;
    (void)upper;
    return KIN_ERR_NOSYS;
}

static void dummy_destroy(kin_solver_t *solver) {
    if (!solver)
            return;

    dummy_priv_t *priv = (dummy_priv_t *)solver->priv_data;
    if (priv) {
            free(priv->urdf_path);
            free(priv->base_link);
            free(priv->tip_link);
            free(priv);
    }

    free((void *)solver->name);
    free(solver);
}

static const struct kin_ops dummy_ops = {
    .forward = dummy_forward,
    .inverse = dummy_inverse,
    .get_num_joints = dummy_get_num_joints,
    .get_joint_limits = dummy_get_joint_limits,
    .destroy = dummy_destroy,
};

/* ==========================================================================
* Factory Function
* ========================================================================== */

static kin_solver_t *dummy_factory(const char *urdf_path,
                                    const char *base_link,
                                    const char *tip_link) {
    kin_solver_t *solver = calloc(1, sizeof(*solver));
    if (!solver)
            return NULL;

    dummy_priv_t *priv = calloc(1, sizeof(*priv));
    if (!priv) {
            free(solver);
            return NULL;
    }

    /* 复制参数 */
    if (urdf_path) {
            priv->urdf_path = strdup(urdf_path);
    }
    if (base_link) {
            priv->base_link = strdup(base_link);
    }
    if (tip_link) {
            priv->tip_link = strdup(tip_link);
    }

    priv->num_joints = 5;  /* SO-101 默认 */

    solver->name = strdup("dummy");
    solver->ops = &dummy_ops;
    solver->priv_data = priv;

    printf("[KIN] Created dummy solver (no IK support)\n");
    return solver;
}

/* 注册 dummy 求解器 */
REGISTER_KIN_SOLVER("dummy", dummy_factory)
