#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
module_root="$(cd "$script_dir/.." && pwd)"
artifact_dir="${SROBOTIS_TEST_ARTIFACT_DIR:-${SROBOTIS_OUTPUT_ROOT:-$PWD/output}/test-artifacts/components/control/manipulator/${SROBOTIS_TEST_NAME:-manipulator-dummy-performance}}"
log_dir="$artifact_dir/logs"
log_file="$log_dir/manipulator_dummy_performance.log"
build_dir="$artifact_dir/build"
bench_bin="$build_dir/benchmark_dummy_manipulator"
max_avg_us="${MANIP_DUMMY_MAX_AVG_US:-300}"
iters="${MANIP_DUMMY_ITERS:-20000}"

mkdir -p "$log_dir" "$build_dir"

{
    echo "[info] module_root=$module_root"
    echo "[info] build_dir=$build_dir"
    echo "[info] MANIP_DUMMY_ITERS=$iters"
    echo "[info] MANIP_DUMMY_MAX_AVG_US=$max_avg_us"

    cmake -S "$module_root" -B "$build_dir" \
        -DMANIP_ENABLE_SO101_DRIVER=OFF \
        -DMANIP_ENABLE_PINOCCHIO=OFF \
        -DMANIP_BUILD_TESTS=OFF \
        -DMANIP_BUILD_BENCHMARKS=ON \
        -DMANIP_BUILD_HW_TEST=OFF

    cmake --build "$build_dir" -j"$(nproc)" --target benchmark_dummy_manipulator

    "$bench_bin" "$iters" "$max_avg_us"
} | tee "$log_file"

grep -q "PERF_OK" "$log_file"
