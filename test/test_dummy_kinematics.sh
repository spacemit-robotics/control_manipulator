#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
module_root="$(cd "$script_dir/.." && pwd)"
artifact_dir="${SROBOTIS_TEST_ARTIFACT_DIR:-${SROBOTIS_OUTPUT_ROOT:-$PWD/output}/test-artifacts/components/control/manipulator/${SROBOTIS_TEST_NAME:-manipulator-dummy-kinematics-functional}}"
log_dir="$artifact_dir/logs"
log_file="$log_dir/manipulator_dummy_kinematics.log"
build_dir="$artifact_dir/build"
test_src="$script_dir/test_dummy_kinematics_api.c"
test_bin="$build_dir/test_dummy_kinematics_api"

mkdir -p "$log_dir" "$build_dir"

{
    echo "[info] module_root=$module_root"
    echo "[info] build_dir=$build_dir"

    cmake -S "$module_root" -B "$build_dir" \
        -DMANIP_ENABLE_SO101_DRIVER=OFF \
        -DMANIP_ENABLE_PINOCCHIO=OFF \
        -DMANIP_BUILD_TESTS=OFF \
        -DMANIP_BUILD_HW_TEST=OFF

    cmake --build "$build_dir" -j"$(nproc)"

    cc -std=c99 -O2 "$test_src" -I"$module_root/include" -L"$build_dir" \
        -Wl,-rpath,"$build_dir" -lmanipulator -lpthread -lm -lstdc++ -o "$test_bin"

    "$test_bin" "$module_root/urdf/so101.urdf"
} | tee "$log_file"

grep -q "DUMMY_KIN_OK" "$log_file"
