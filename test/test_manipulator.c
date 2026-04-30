/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file test_manipulator.c
 * @brief Unit test for manipulator framework
 *
 * Tests the core manipulator API using the dummy driver (drv_dummy.c).
 * The dummy driver simulates instant motion completion without hardware.
 *
 * For hardware testing with real robots (e.g., SO-101), see:
 *   - test_hw_so101.c: Interactive hardware test program
 *   - HW_TEST_GUIDE.md: Hardware test guide
 */

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "manipulator.h"

static void test_dummy_basic(void) {
    /* Test dummy driver (src/drivers/drv_dummy.c)
   * Dummy driver simulates instant completion, no hardware required */

    /* 1. Alloc */
    struct manip_dev *dev = manip_alloc("dummy", NULL);
    assert(dev != NULL);

    /* 2. Move joints */
    manip_joint_t tgt = {.count = 3, .joints = {0.1f, 0.2f, 0.3f}};
    int ret = manip_move_joints(dev, &tgt, 0.5f);
    assert(ret == MANIP_OK);

    /* 3. Get state - should reflect what we set */
    manip_joint_t state;
    memset(&state, 0, sizeof(state));
    ret = manip_get_state(dev, &state, NULL);
    assert(ret == MANIP_OK);
    assert(state.count == 3);
    assert(fabsf(state.joints[0] - 0.1f) < 1e-6f);
    assert(fabsf(state.joints[1] - 0.2f) < 1e-6f);
    assert(fabsf(state.joints[2] - 0.3f) < 1e-6f);

    /* 4. Tick should transition from MOVING to IDLE */
    manip_tick(dev, 0.005f);

    /* 5. move_line should return NOSYS (not implemented) */
    manip_pose_t pose = {0};
    ret = manip_move_line(dev, &pose, 1.0f);
    assert(ret == MANIP_ERR_NOSYS);

    /* 6. set_tcp should return NOSYS */
    ret = manip_set_tcp(dev, &pose);
    assert(ret == MANIP_ERR_NOSYS);

    /* 7. set_teach_mode should return NOSYS */
    ret = manip_set_teach_mode(dev, true);
    assert(ret == MANIP_ERR_NOSYS);

    /* 8. Stop */
    manip_stop(dev);

    /* 9. Invalid driver name */
    struct manip_dev *bad = manip_alloc("nonexistent", NULL);
    assert(bad == NULL);

    /* 10. NULL checks */
    assert(manip_move_joints(NULL, &tgt, 0.5f) == MANIP_ERR_PARAM);
    assert(manip_get_state(NULL, &state, NULL) == MANIP_ERR_PARAM);

    /* Cleanup */
    manip_free(dev);
    printf("=== manipulator test PASSED ===\n");
}

int main(void) {
    test_dummy_basic();
    return 0;
}
