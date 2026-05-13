/*
        * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
        * SPDX-License-Identifier: Apache-2.0
        */

/*
        * CAN 电机测试程序（统一入口）
        * - 仅用于 CAN 类型电机
        * - 默认覆盖原 test_damiao_can.c 的功能（dm_can + HYBRID 控制循环）
        */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "../include/motor.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static void print_usage(const char *prog) {
        printf("Usage: %s [options]\n", prog);
        printf("\n");
        printf("Options:\n");
        printf("  --driver <name>     motor driver name (default: drv_can_dm)\n");
        printf("  --if <canX>         CAN interface (default: can0)\n");
        printf("  --id <hex|dec>      motor id; can be specified multiple times\n");
        printf("  --loops <n>         loop count (default: 100)\n");
        printf("  --period_ms <n>     loop period in ms (default: 50)\n");
        printf("  --help              show this help\n");
        printf("\n");
        printf("Examples:\n");
        printf("  %s\n", prog);
        printf("  %s --driver drv_can_dm --if can0 --id 0x02 --id 0x03\n", prog);
        printf("  %s --driver dji_m3508 --if can0 --id 0x201 --id 0x202\n", prog);
}

static int parse_int_auto_base(const char *s) {
        char *end = NULL;
        int64_t v = strtol(s, &end, 0);
        if (!s || *s == '\0' || (end && *end != '\0')) {
                return -1;
        }
        if (v < 0 || v > 0x7FFFFFFF) {
                return -1;
        }
        return (int)v;
}

int main(int argc, char **argv) {
        const char *driver = "drv_can_dm";
        const char *can_if = "can0";
        int loops = 500;     // 增加默认循环次数
        int period_ms = 10;  // 提高默认控制频率 (100Hz)

        int ids_cap = 4;
        int ids_cnt = 0;
        int *ids = (int *)calloc((size_t)ids_cap, sizeof(int));
        if (!ids) {
                printf("错误：内存分配失败\n");
                return -1;
        }

        for (int i = 1; i < argc; i++) {
                if (strcmp(argv[i], "--help") == 0) {
                        print_usage(argv[0]);
                        free(ids);
                        return 0;
                } else if (strcmp(argv[i], "--driver") == 0 && i + 1 < argc) {
                        driver = argv[++i];
                } else if (strcmp(argv[i], "--if") == 0 && i + 1 < argc) {
                        can_if = argv[++i];
                } else if (strcmp(argv[i], "--loops") == 0 && i + 1 < argc) {
                        loops = parse_int_auto_base(argv[++i]);
                } else if (strcmp(argv[i], "--period_ms") == 0 && i + 1 < argc) {
                        period_ms = parse_int_auto_base(argv[++i]);
                } else if (strcmp(argv[i], "--id") == 0 && i + 1 < argc) {
                        int id = parse_int_auto_base(argv[++i]);
                        if (id < 0) {
                                printf("错误：非法 id 参数\n");
                                print_usage(argv[0]);
                                free(ids);
                                return -1;
                        }
                        if (ids_cnt >= ids_cap) {
                                ids_cap *= 2;
                                int *new_ids = (int *)realloc(ids, (size_t)ids_cap * sizeof(int));
                                if (!new_ids) {
                                        printf("错误：内存分配失败\n");
                                        free(ids);
                                        return -1;
                                }
                                ids = new_ids;
                        }
                        ids[ids_cnt++] = id;
                } else {
                        printf("错误：未知参数 %s\n", argv[i]);
                        print_usage(argv[0]);
                        free(ids);
                        return -1;
                }
        }

        if (loops <= 0) {
                printf("错误：--loops 必须 > 0\n");
                free(ids);
                return -1;
        }
        if (period_ms <= 0) {
                printf("错误：--period_ms 必须 > 0\n");
                free(ids);
                return -1;
        }

        /* Default to original damiao test IDs if not provided. */
        if (ids_cnt == 0) {
                ids[ids_cnt++] = 0x02;
                ids[ids_cnt++] = 0x03;
        }

        printf("=== CAN 电机测试程序 ===\n");
        printf("driver=%s, if=%s, motors=%d, loops=%d, period_ms=%d\n\n", driver, can_if, ids_cnt,
                        loops, period_ms);

        struct motor_dev **motors = (struct motor_dev **)calloc((size_t)ids_cnt, sizeof(*motors));
        struct motor_state *states = (struct motor_state *)calloc((size_t)ids_cnt, sizeof(*states));
        struct motor_cmd *cmds = (struct motor_cmd *)calloc((size_t)ids_cnt, sizeof(*cmds));
        if (!motors || !states || !cmds) {
                printf("错误：内存分配失败\n");
                free(motors);
                free(states);
                free(cmds);
                free(ids);
                return -1;
        }

        printf("1. 分配 CAN 电机设备...\n");
        for (int i = 0; i < ids_cnt; i++) {
                motors[i] = motor_alloc_can(driver, can_if, ids[i], NULL);
                if (!motors[i]) {
                        printf("错误：电机分配失败 (index=%d, id=%d)\n", i, ids[i]);
                        for (int k = 0; k < ids_cnt; k++) {
                                if (motors[k]) {
                                        motor_free(&motors[k], 1);
                                }
                        }
                        free(motors);
                        free(states);
                        free(cmds);
                        free(ids);
                        return -1;
                }
        }

        printf("2. 初始化电机...\n");
        int ret = motor_init(motors, ids_cnt);
        if (ret < 0) {
                printf("错误：电机初始化失败 (ret=%d)\n", ret);
                motor_free(motors, ids_cnt);
                free(motors);
                free(states);
                free(cmds);
                free(ids);
                return -1;
        }

        printf("初始化成功！开始控制循环...\n\n");

        float total_time = (float)loops * (float)period_ms / 1000.0f;
        float omega = 3.0f * 2.0f * (float)M_PI / total_time;

        for (int i = 0; i < loops; i++) {
                float t = (float)i * (float)period_ms / 1000.0f;

                /* 计算平滑的正弦往返运动系数 (0.0 -> 1.0 -> 0.0, 重复 3 次) */
                /* 使用 pos = A/2 * (1 - cos(omega * t)) 消除换向瞬间的速度阶跃 */
                float coefficient = 0.5f * (1.0f - cosf(omega * t));
                /* 速度前馈 v = d(pos)/dt = A/2 * omega * sin(omega * t) */
                float v_coeff = 0.5f * omega * sinf(omega * t);

                /* 默认: 模仿原始 damiao CAN 测试 (HYBRID 模式) */
                for (int j = 0; j < ids_cnt; j++) {
                        float max_pos = 3.7f * (float)(j + 1);
                        cmds[j].mode = MOTOR_MODE_HYBRID;
                        // 目标位置随系数波动
                        cmds[j].pos_des = max_pos * coefficient;
                        // 引入速度前馈，显著提升跟随平滑度
                        cmds[j].vel_des = max_pos * v_coeff;
                        cmds[j].trq_des = 0.0f;
                        cmds[j].kp = 10.0f;
                        cmds[j].kd = 1.0f;
                }

                ret = motor_set_cmds(motors, cmds, ids_cnt);
                if (ret < 0) {
                        printf("警告：发送控制命令失败 (ret=%d)\n", ret);
                }

                ret = motor_get_states(motors, states, ids_cnt);
                if (ret == 0 && (i % 10 == 0)) {
                        printf("循环 %d: ", i);
                        for (int j = 0; j < ids_cnt; j++) {
                                printf("M%d(%.2f) ", j + 1, states[j].pos);
                        }
                        printf("\n");
                }

                usleep((useconds_t)period_ms * 1000U);
        }

        printf("\n3. 清理资源...\n");
        motor_free(motors, ids_cnt);

        free(motors);
        free(states);
        free(cmds);
        free(ids);

        printf("测试完成！\n");
        return 0;
}
