#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
module_root="$(cd "$script_dir/.." && pwd)"
artifact_dir="${SROBOTIS_TEST_ARTIFACT_DIR:-${SROBOTIS_OUTPUT_ROOT:-$PWD/output}/test-artifacts/components/control/manipulator/${SROBOTIS_TEST_NAME:-manipulator-dummy-performance}}"
log_dir="$artifact_dir/logs"
log_file="$log_dir/manipulator_dummy_performance.log"
build_dir="$artifact_dir/build"
bench_src="$script_dir/benchmark_dummy_manipulator.c"
bench_bin="$build_dir/benchmark_dummy_manipulator"
bench_log="$log_dir/benchmark_dummy_manipulator_build.log"
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
        -DMANIP_BUILD_HW_TEST=OFF

    cmake --build "$build_dir" -j"$(nproc)"

    c++ -x c -std=c99 -O2 "$bench_src" -I"$module_root/include" -L"$build_dir" \
        -Wl,-rpath,"$build_dir" -lmanipulator -lpthread -lm -o "$bench_bin" \
        >"$bench_log" 2>&1

    echo "[info] benchmark build log: $bench_log"
    cat "$bench_log"

    "$bench_bin" "$iters" "$max_avg_us"
} | tee "$log_file"

grep -q "PERF_OK" "$log_file"
