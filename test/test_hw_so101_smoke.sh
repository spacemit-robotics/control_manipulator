#!/usr/bin/env bash
# Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
# SPDX-License-Identifier: Apache-2.0
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
module_root="$(cd "$script_dir/.." && pwd)"
artifact_dir="${SROBOTIS_TEST_ARTIFACT_DIR:-${SROBOTIS_OUTPUT_ROOT:-$PWD/output}/test-artifacts/components/control/manipulator/${SROBOTIS_TEST_NAME:-manipulator-so101-hardware-smoke}}"
log_dir="$artifact_dir/logs"
log_file="$log_dir/manipulator_so101_hardware_smoke.log"
build_dir="$artifact_dir/build"
port="${MANIP_SO101_PORT:-/dev/ttyACM0}"

mkdir -p "$log_dir" "$build_dir"

{
    echo "[info] module_root=$module_root"
    echo "[info] build_dir=$build_dir"
    echo "[info] port=$port"

    test -e "$port"

    cmake -S "$module_root" -B "$build_dir" \
        -DMANIP_BUILD_TESTS=OFF \
        -DMANIP_BUILD_HW_TEST=ON

    cmake --build "$build_dir" -j"$(nproc)"

    printf '0\n' | "$build_dir/test_hw_so101" "$port"
} | tee "$log_file"

grep -q "SO-101 机械臂 + 夹爪 硬件测试程序" "$log_file"
