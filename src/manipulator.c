/*
* Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
* SPDX-License-Identifier: Apache-2.0
*
* @file manipulator.c
* @brief Core implementation for manipulator control component
*/

#include "manipulator_core.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ==========================================================================
* Driver Registry (linked list, same pattern as chassis)
* ========================================================================== */

static struct manip_driver_info *g_driver_list = NULL;

void manip_driver_register(struct manip_driver_info *info) {
    if (!info)
        return;
    info->next = g_driver_list;
    g_driver_list = info;
    printf("[MANIP] Registered driver: %s\n", info->name);
}

static struct manip_driver_info *find_driver(const char *name) {
    struct manip_driver_info *curr = g_driver_list;

    while (curr) {
        if (curr->name && name && strcmp(curr->name, name) == 0)
            return curr;
        curr = curr->next;
    }

    printf("[MANIP] Driver not found: %s\n", name ? name : "(null)");
    return NULL;
}

/* ==========================================================================
* Device Allocation Helper
* ========================================================================== */

struct manip_dev *manip_dev_alloc(const char *name, size_t priv_size) {
    struct manip_dev *dev = calloc(1, sizeof(*dev));
    if (!dev)
        return NULL;

    if (priv_size) {
        dev->priv_data = calloc(1, priv_size);
        if (!dev->priv_data) {
            free(dev);
            return NULL;
        }
    }

    if (name) {
        dev->name = strdup(name);
        if (!dev->name) {
            free(dev->priv_data);
            free(dev);
            return NULL;
        }
    }

    dev->state = MANIP_IDLE;
    pthread_mutex_init(&dev->state_lock, NULL);
    return dev;
}

void manip_dev_free_default(struct manip_dev *dev) {
    if (!dev)
        return;

    /* Release kinematics solver if attached */
    if (dev->kin_solver) {
        kin_destroy(dev->kin_solver);
        dev->kin_solver = NULL;
    }

    pthread_mutex_destroy(&dev->state_lock);

    if (dev->priv_data)
        free(dev->priv_data);
    if (dev->name)
        free((void *)dev->name);
    free(dev);
}

/* ==========================================================================
* Public API Implementation
* ========================================================================== */

struct manip_dev *manip_alloc(const char *driver_name, void *args) {
    struct manip_driver_info *drv;

    if (!driver_name)
        return NULL;

    drv = find_driver(driver_name);
    if (!drv || !drv->factory) {
        printf("[MANIP] No driver found: %s\n", driver_name);
        return NULL;
    }

    return drv->factory(driver_name, args);
}

int manip_move_joints(struct manip_dev *dev, const manip_joint_t *target,
                                            float speed_ratio) {
    if (!dev || !target)
        return MANIP_ERR_PARAM;

    if (dev->ops && dev->ops->move_joints)
        return dev->ops->move_joints(dev, target, speed_ratio);

    return MANIP_ERR_NOSYS;
}

int manip_move_line(struct manip_dev *dev, const manip_pose_t *target,
                                        float speed_ratio) {
    if (!dev || !target)
        return MANIP_ERR_PARAM;

    /* 1. 驱动原生 move_line 优先 */
    if (dev->ops && dev->ops->move_line)
        return dev->ops->move_line(dev, target, speed_ratio);

    /* 2. 框架级 fallback：IK → move_joints */
    if (dev->kin_solver && dev->ops && dev->ops->move_joints) {
        kin_pose_t ik_target;
        ik_target.x  = target->x;
        ik_target.y  = target->y;
        ik_target.z  = target->z;
        ik_target.qw = target->qw;
        ik_target.qx = target->qx;
        ik_target.qy = target->qy;
        ik_target.qz = target->qz;

        /* 用当前关节角作为 IK 种子值 */
        kin_joints_t q_init;
        q_init.count = dev->cur_joints.count;
        for (int i = 0; i < q_init.count && i < KIN_MAX_JOINTS; i++) {
            q_init.q[i] = (double)dev->cur_joints.joints[i];
        }

        kin_joints_t q_result;
        int ik_ret = kin_inverse(dev->kin_solver, &ik_target, &q_init,
                                                            NULL, &q_result);
        if (ik_ret != KIN_OK) {
            printf("[MANIP] IK failed for move_line (error %d)\n", ik_ret);
            return MANIP_ERR_NOSYS;
        }

        /* IK 结果 → manip_joint_t */
        manip_joint_t joint_target;
        joint_target.count = q_result.count;
        for (int i = 0; i < q_result.count && i < MANIP_MAX_JOINTS; i++) {
            joint_target.joints[i] = (float)q_result.q[i];
        }

        return dev->ops->move_joints(dev, &joint_target, speed_ratio);
    }

    return MANIP_ERR_NOSYS;
}

int manip_move_target(struct manip_dev *dev, const manip_pose_t *target,
                        float speed_ratio) {
    if (!dev || !target)
        return MANIP_ERR_PARAM;

    manip_joint_t joint_target;
    int ret = manip_solve_target_joints(dev, target, &joint_target);
    if (ret != MANIP_OK)
        return ret;

    return dev->ops->move_joints(dev, &joint_target, speed_ratio);
}

int manip_solve_target_joints(struct manip_dev *dev,
                                const manip_pose_t *target,
                                manip_joint_t *out_joints) {
    if (!dev || !target || !out_joints)
        return MANIP_ERR_PARAM;

    if (!dev->kin_solver || !dev->ops || !dev->ops->move_joints)
        return MANIP_ERR_NOSYS;

    kin_pose_t ik_target;
    ik_target.x  = target->x;
    ik_target.y  = target->y;
    ik_target.z  = target->z;
    ik_target.qw = target->qw;
    ik_target.qx = target->qx;
    ik_target.qy = target->qy;
    ik_target.qz = target->qz;

    kin_joints_t q_init;
    q_init.count = dev->cur_joints.count;
    for (int i = 0; i < q_init.count && i < KIN_MAX_JOINTS; i++)
        q_init.q[i] = (double)dev->cur_joints.joints[i];

    kin_ik_params_t ik_params = {
        .epsilon = 1e-3,
        .position_weight = 1,
        .timeout_s = 0.1,
    };

    kin_joints_t q_result;
    int ik_ret = kin_inverse(dev->kin_solver, &ik_target, &q_init,
                                &ik_params, &q_result);
    if (ik_ret != KIN_OK)
        return MANIP_ERR_NOSYS;

    out_joints->count = q_result.count;
    for (int i = 0; i < q_result.count && i < MANIP_MAX_JOINTS; i++)
        out_joints->joints[i] = (float)q_result.q[i];

    return MANIP_OK;
}

void manip_stop(struct manip_dev *dev) {
    if (!dev)
        return;

    if (dev->ops && dev->ops->stop)
        dev->ops->stop(dev);
}

int manip_set_tcp(struct manip_dev *dev, const manip_pose_t *tcp_offset) {
    if (!dev || !tcp_offset)
        return MANIP_ERR_PARAM;

    if (dev->ops && dev->ops->set_tcp)
        return dev->ops->set_tcp(dev, tcp_offset);

    return MANIP_ERR_NOSYS;
}

int manip_set_teach_mode(struct manip_dev *dev, bool enable) {
    if (!dev)
        return MANIP_ERR_PARAM;

    if (dev->ops && dev->ops->set_teach_mode)
        return dev->ops->set_teach_mode(dev, enable);

    return MANIP_ERR_NOSYS;
}

int manip_get_state(struct manip_dev *dev, manip_joint_t *out_joints,
                                        manip_pose_t *out_pose) {
    if (!dev)
        return MANIP_ERR_PARAM;

    if (dev->ops && dev->ops->get_state) {
        int ret = dev->ops->get_state(dev, out_joints, out_pose);

        /* 如果驱动返回了 joints 但没有 pose，尝试用 FK 计算 */
        if (ret == MANIP_OK && out_pose && dev->kin_solver && out_joints) {
            kin_joints_t fk_in;
            fk_in.count = out_joints->count;
            for (int i = 0; i < fk_in.count && i < KIN_MAX_JOINTS; i++) {
                fk_in.q[i] = (double)out_joints->joints[i];
            }

            kin_pose_t fk_out;
            if (kin_forward(dev->kin_solver, &fk_in, &fk_out) == KIN_OK) {
                out_pose->x  = (float)fk_out.x;
                out_pose->y  = (float)fk_out.y;
                out_pose->z  = (float)fk_out.z;
                out_pose->qw = (float)fk_out.qw;
                out_pose->qx = (float)fk_out.qx;
                out_pose->qy = (float)fk_out.qy;
                out_pose->qz = (float)fk_out.qz;
            }
        }

        return ret;
    }

    return MANIP_ERR_NOSYS;
}

void manip_tick(struct manip_dev *dev, float dt_s) {
    if (!dev)
        return;

    if (dev->ops && dev->ops->tick)
        dev->ops->tick(dev, dt_s);
}

void manip_free(struct manip_dev *dev) {
    if (!dev)
        return;

    if (dev->running) {
        dev->running = false;
        if (dev->ops && dev->ops->stop)
            dev->ops->stop(dev);
    }

    if (dev->ops && dev->ops->free) {
        dev->ops->free(dev);
        return;
    }

    manip_dev_free_default(dev);
}

int manip_set_kinematics(struct manip_dev *dev, struct kin_solver *solver) {
    if (!dev)
        return MANIP_ERR_PARAM;

    /* Release previous solver if any */
    if (dev->kin_solver && dev->kin_solver != solver)
        kin_destroy(dev->kin_solver);

    dev->kin_solver = solver;
    return MANIP_OK;
}
