/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <stdlib.h>

#include "kinematics_interface.h"

int main(int argc, char **argv) {
    const char *urdf_path = argc > 1 ? argv[1] : "../urdf/so101.urdf";
    kin_solver_t *solver = kin_create("dummy", urdf_path, "base_link", "gripper_frame_link");
    if (!solver) {
        fprintf(stderr, "failed to create dummy solver\n");
        return 1;
    }

    int joints = kin_get_num_joints(solver);
    if (joints != 5) {
        fprintf(stderr, "unexpected joint count=%d\n", joints);
        kin_destroy(solver);
        return 2;
    }

    kin_joints_t q = {0};
    q.count = 5;
    kin_pose_t pose = {0};
    if (kin_forward(solver, &q, &pose) != KIN_ERR_NOSYS) {
        fprintf(stderr, "expected KIN_ERR_NOSYS from kin_forward\n");
        kin_destroy(solver);
        return 3;
    }

    kin_joints_t out = {0};
    if (kin_inverse(solver, &pose, NULL, NULL, &out) != KIN_ERR_NOSYS) {
        fprintf(stderr, "expected KIN_ERR_NOSYS from kin_inverse\n");
        kin_destroy(solver);
        return 4;
    }

    printf("DUMMY_KIN_OK joints=%d\n", joints);
    kin_destroy(solver);
    return 0;
}
