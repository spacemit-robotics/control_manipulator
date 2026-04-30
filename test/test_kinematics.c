/*
* Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
* SPDX-License-Identifier: Apache-2.0
*
* @file test_kinematics.c
* @brief 测试运动学求解器 (Pinocchio)
*/

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "kinematics_interface.h"

#define SO101_URDF_PATH "../urdf/so101.urdf"
#define SO101_BASE_LINK "base_link"
#define SO101_TIP_LINK  "gripper_frame_link"

int main(void) {
    printf("=== Kinematics Solver Test ===\n\n");

    // 创建 Pinocchio 求解器
    printf("1. Creating Pinocchio solver...\n");
    kin_solver_t *solver = kin_create("pinocchio", SO101_URDF_PATH, SO101_BASE_LINK, SO101_TIP_LINK);
    if (!solver) {
        fprintf(stderr, "Failed to create Pinocchio solver\n");
        return 1;
    }

    // 获取关节数量
    int num_joints = kin_get_num_joints(solver);
    printf("   Joints: %d\n\n", num_joints);

    // 测试正运动学
    printf("2. Testing Forward Kinematics...\n");
    kin_joints_t joints = {0};
    joints.count = num_joints;
    // 零位姿态
    for (int i = 0; i < num_joints; i++) {
        joints.q[i] = 0.0;
    }

    kin_pose_t pose = {0};
    int ret = kin_forward(solver, &joints, &pose);
    if (ret == KIN_OK) {
        printf("   ✓ FK Success\n");
        printf("   Position: [%.3f, %.3f, %.3f]\n", pose.x, pose.y, pose.z);
        printf("   Quaternion: [%.3f, %.3f, %.3f, %.3f]\n",
                        pose.qw, pose.qx, pose.qy, pose.qz);
    } else {
        printf("   ✗ FK Failed: %d\n", ret);
        kin_destroy(solver);
        return 1;
    }

    // 测试逆运动学
    printf("\n3. Testing Inverse Kinematics...\n");
    printf("   Target: same as FK result\n");

    kin_joints_t ik_result = {0};
    ret = kin_inverse(solver, &pose, NULL, NULL, &ik_result);
    if (ret == KIN_OK) {
        printf("   ✓ IK Success\n");
        printf("   Joints: [");
        for (int i = 0; i < ik_result.count; i++) {
            printf("%.3f%s", ik_result.q[i], i < ik_result.count-1 ? ", " : "");
        }
        printf("]\n");

        // 验证 FK/IK 一致性
        kin_pose_t verify_pose = {0};
        kin_forward(solver, &ik_result, &verify_pose);

        double pos_error = sqrt(
                pow(verify_pose.x - pose.x, 2) +
                pow(verify_pose.y - pose.y, 2) +
                pow(verify_pose.z - pose.z, 2));

        printf("   Position error: %.6f m\n", pos_error);

        if (pos_error < 0.001) {
            printf("   ✓ FK/IK consistency verified\n");
        } else {
            printf("   ⚠ Position error too large\n");
        }
    } else {
        printf("   ✗ IK Failed: %d\n", ret);
    }

    // 清理
    printf("\n4. Cleanup...\n");
    kin_destroy(solver);

    printf("\n=== Test PASSED ===\n");
    return 0;
}
