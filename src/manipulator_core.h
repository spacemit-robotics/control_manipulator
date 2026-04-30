/*
* Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
* SPDX-License-Identifier: Apache-2.0
*/
#ifndef MANIPULATOR_CORE_H
#define MANIPULATOR_CORE_H

/**
* @file manipulator_core.h
* @brief Private header for manipulator control component (internal use only)
*/

#include <pthread.h>
#include <stddef.h>

#include "../include/manipulator.h"
#include "../include/kinematics_interface.h"

/* ==========================================================================
* 1. Virtual Operations Table (driver interface)
* ========================================================================== */

struct manip_ops {
    int (*move_joints)(struct manip_dev *dev, const manip_joint_t *target,
                                            float speed_ratio);
    int (*move_line)(struct manip_dev *dev, const manip_pose_t *target,
                                        float speed_ratio);
    void (*stop)(struct manip_dev *dev);
    int (*set_tcp)(struct manip_dev *dev, const manip_pose_t *tcp_offset);
    int (*set_teach_mode)(struct manip_dev *dev, bool enable);
    int (*get_state)(struct manip_dev *dev, manip_joint_t *out_joints,
                                        manip_pose_t *out_pose);
    void (*tick)(struct manip_dev *dev, float dt_s);
    void (*free)(struct manip_dev *dev);
};

/* ==========================================================================
* 2. Device Structure (internal)
* ========================================================================== */

struct manip_dev {
    /* Identity */
    const char *name;

    /* Operations */
    const struct manip_ops *ops;
    void *priv_data;

    /* Kinematics solver (optional, may be NULL) */
    kin_solver_t *kin_solver;

    /* State (protected by lock) */
    manip_joint_t cur_joints;
    manip_pose_t cur_pose;
    manip_state_t state;
    pthread_mutex_t state_lock;

    /* Runtime */
    bool running;
};

/* ==========================================================================
* 3. Driver Registry
* ========================================================================== */

typedef struct manip_dev *(*manip_factory_t)(const char *name, void *args);

struct manip_driver_info {
    const char *name;
    manip_factory_t factory;
    struct manip_driver_info *next;
};

void manip_driver_register(struct manip_driver_info *info);

/**
* @brief Register a manipulator driver (called at load time via constructor)
* @param _name    Driver name string
* @param _factory Factory function
*/
#define REGISTER_MANIP_DRIVER(_name, _factory)                                 \
    static struct manip_driver_info __drv_info_##_factory = {  \
            .name = _name, .factory = _factory, .next = NULL};  \
    __attribute__((constructor)) static void __auto_reg_##_factory(void) {  \
        manip_driver_register(&__drv_info_##_factory);  \
    }

/* ==========================================================================
    * 4. Internal Helpers
    * ========================================================================== */

/**
* @brief Allocate a manipulator device with private data
*/
struct manip_dev *manip_dev_alloc(const char *name, size_t priv_size);

/**
* @brief Default free implementation
*/
void manip_dev_free_default(struct manip_dev *dev);

#endif  // MANIPULATOR_CORE_H
