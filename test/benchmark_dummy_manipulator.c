/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */

#define _POSIX_C_SOURCE 200809L

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "manipulator.h"

static uint64_t monotonic_ns(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ULL + (uint64_t)ts.tv_nsec;
}

int main(int argc, char **argv) {
    int iters = 20000;
    double max_avg_us = 300.0;
    if (argc > 1) {
        iters = atoi(argv[1]);
    }
    if (argc > 2) {
        max_avg_us = atof(argv[2]);
    }

    struct manip_dev *dev = manip_alloc("dummy", NULL);
    if (!dev) {
        fprintf(stderr, "failed to alloc dummy manipulator\n");
        return 1;
    }

    manip_joint_t joints;
    memset(&joints, 0, sizeof(joints));
    joints.count = 5;

    uint64_t start = monotonic_ns();
    for (int i = 0; i < iters; ++i) {
        joints.joints[0] = (float)(i % 100) * 0.001f;
        joints.joints[1] = (float)(i % 50) * -0.001f;
        if (manip_move_joints(dev, &joints, 0.5f) != MANIP_OK) {
            fprintf(stderr, "manip_move_joints failed at iter=%d\n", i);
            manip_free(dev);
            return 2;
        }
        manip_tick(dev, 0.001f);
    }
    uint64_t end = monotonic_ns();

    double total_us = (double)(end - start) / 1000.0;
    double avg_us = total_us / (double)iters;
    printf("iters=%d total_us=%.3f avg_us=%.3f threshold_us=%.3f\n", iters, total_us, avg_us, max_avg_us);
    if (avg_us > max_avg_us) {
        fprintf(stderr, "PERF_FAIL avg_us=%.3f threshold_us=%.3f\n", avg_us, max_avg_us);
        manip_free(dev);
        return 3;
    }

    printf("PERF_OK avg_us=%.3f threshold_us=%.3f\n", avg_us, max_avg_us);
    manip_free(dev);
    return 0;
}
