/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>

#include "motor.h"

/**
 * @brief Reachy Mini Motor Driver Example
 *
 * This example demonstrates how to use the "reachy_mini" driver to control
 * the robot"s motors (Body Yaw, Stewart Platform, and Antennas).
 * It implements a safe sine-wave trajectory (±30 degrees) to ensure the
 * movements are within mechanical limits.
 */

#define MOTOR_COUNT 9
#define BAUDRATE 1000000
#define DEFAULT_PORT "/dev/ttyACM0"

// Safe parameters from sin.py
#define AMP_DEG 30.0f
#define FREQ_HZ 0.25f
#define DURATION_S 10.0f
#define CONTROL_PERIOD_US 20000  // 50Hz

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

int main(int argc, char *argv[]) {
    const char *port = DEFAULT_PORT;
    if (argc >= 3) {
        port = argv[2];
    } else if (argc >= 2) {
        port = argv[1];
    }

    struct motor_dev *devs[MOTOR_COUNT];
    struct motor_cmd cmds[MOTOR_COUNT];
    struct motor_state states[MOTOR_COUNT];

    printf("[Test] Initializing %d Reachy Mini motors on %s...\n", MOTOR_COUNT,
        port);

    /* 1. Allocate motor devices (IDs 10-18)
     * ID 10: Body Yaw
     * ID 11-16: Stewart Platform Legs 1-6
     * ID 17-18: Antennas Right/Left
     */
    for (int i = 0; i < MOTOR_COUNT; i++) {
        uint8_t id = 10 + i;
        devs[i] = motor_alloc_uart("drv_uart_rm", port, BAUDRATE, id, NULL);
        if (!devs[i]) {
            fprintf(stderr, "Error: Failed to allocate motor ID %d\n", id);
            // Cleanup previously allocated
            motor_free(devs, i);
            return -1;
        }
    }

    /* 2. Initialize motors (set to active mode) */
    if (motor_init(devs, MOTOR_COUNT) != 0) {
        fprintf(stderr, "Error: Failed to initialize motors\n");
        motor_free(devs, MOTOR_COUNT);
        return -1;
    }

    printf("[Test] Motors initialized. Starting sine-wave motion...\n");
    printf("[Test] Amplitude: %.1f deg, Frequency: %.2f Hz, Duration: %.1f s\n",
        AMP_DEG, FREQ_HZ, DURATION_S);

    /* 3. Prepare common command fields */
    for (int i = 0; i < MOTOR_COUNT; i++) {
        cmds[i].mode = MOTOR_MODE_POS;
        cmds[i].vel_des = 0.0f;
        cmds[i].trq_des = 0.0f;
        cmds[i].kp = 0.0f;
        cmds[i].kd = 0.0f;
    }

    struct timespec start_time, current_time;
    clock_gettime(CLOCK_MONOTONIC, &start_time);

    float amp_rad = (float)(AMP_DEG * M_PI / 180.0);

    /* 4. Control Loop */
    while (1) {
        clock_gettime(CLOCK_MONOTONIC, &current_time);
        double elapsed = (current_time.tv_sec - start_time.tv_sec) +
            (current_time.tv_nsec - start_time.tv_nsec) * 1e-9;

        if (elapsed > DURATION_S) {
            break;
        }

        // Calculate target position (Sine wave)
        float target_pos = amp_rad * sinf((float)(2.0 * M_PI * FREQ_HZ * elapsed));

        for (int i = 0; i < MOTOR_COUNT; i++) {
            cmds[i].pos_des = target_pos;
        }

        // Send commands to all motors
        motor_set_cmds(devs, cmds, MOTOR_COUNT);

        // Read back states
        if (motor_get_states(devs, states, MOTOR_COUNT) == 0) {
            printf("\rTime: %5.2fs | Goal: %6.3f rad | BodyYaw: %6.3f | Leg1: %6.3f",
                elapsed, target_pos, states[0].pos, states[1].pos);
            if (states[0].err != 0 || states[1].err != 0) {
                printf(" | ERR: 0x%X", states[0].err | states[1].err);
            }
            fflush(stdout);
        }

        usleep(CONTROL_PERIOD_US);
    }

    printf("\n[Test] Demo finished. Moving back to zero position...\n");

    /* 5. Return to zero and cleanup */
    for (int i = 0; i < MOTOR_COUNT; i++) {
        cmds[i].pos_des = 0.0f;
    }
    motor_set_cmds(devs, cmds, MOTOR_COUNT);
    sleep(1);

    motor_free(devs, MOTOR_COUNT);
    printf("[Test] All devices released.\n");

    return 0;
}
