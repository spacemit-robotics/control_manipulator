/*
* Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
* SPDX-License-Identifier: Apache-2.0
*
* @file drv_dummy.c
* @brief Dummy manipulator driver (stub implementation)
*
* 当没有真实硬件驱动时，使用此 dummy 实现。
* 所有运动命令在下一次 tick() 时立即完成。
*/

#include <stdio.h>
#include <string.h>

#include "../manipulator_core.h"

/* ==========================================================================
* Dummy Driver Operations
* ========================================================================== */

static int dummy_move_joints(struct manip_dev *dev, const manip_joint_t *t,
                                                            float s) {
    (void)s;
    pthread_mutex_lock(&dev->state_lock);
    if (t && t->count <= MANIP_MAX_JOINTS) {
        dev->cur_joints = *t;
        dev->state = MANIP_MOVING;
    }
    pthread_mutex_unlock(&dev->state_lock);
    return MANIP_OK;
}

static void dummy_stop(struct manip_dev *dev) {
    pthread_mutex_lock(&dev->state_lock);
    dev->state = MANIP_IDLE;
    pthread_mutex_unlock(&dev->state_lock);
}

static int dummy_get_state(struct manip_dev *dev,
                            manip_joint_t *out_joints,
                            manip_pose_t *out_pose) {
    pthread_mutex_lock(&dev->state_lock);
    if (out_joints)
        *out_joints = dev->cur_joints;
    if (out_pose)
        *out_pose = dev->cur_pose;
    pthread_mutex_unlock(&dev->state_lock);
    return MANIP_OK;
}

static void dummy_tick(struct manip_dev *dev, float dt_s) {
    (void)dt_s;
    pthread_mutex_lock(&dev->state_lock);
    /* Simulate instant completion */
    if (dev->state == MANIP_MOVING)
        dev->state = MANIP_IDLE;
    pthread_mutex_unlock(&dev->state_lock);
}

static const struct manip_ops dummy_ops = {
    .move_joints = dummy_move_joints,
    .move_line = NULL,
    .stop = dummy_stop,
    .set_tcp = NULL,
    .set_teach_mode = NULL,
    .get_state = dummy_get_state,
    .tick = dummy_tick,
    .free = NULL, /* use default free */
};

static struct manip_dev *dummy_factory(const char *name, void *args) {
    (void)args;
    struct manip_dev *dev = manip_dev_alloc(name, 0);
    if (!dev)
        return NULL;
    dev->ops = &dummy_ops;
    return dev;
}

REGISTER_MANIP_DRIVER("dummy", dummy_factory)
