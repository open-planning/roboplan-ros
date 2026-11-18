#!/usr/bin/env bash
# Temporary diagnostics for the Ubuntu 26.04 test crashes.
# Usage: debug_linux_crash.sh <pixi-environment>, from the repository root.
set -uo pipefail

TESTS=(test_joints test_scene)

section() {
  if [[ -n "${GITHUB_ACTIONS:-}" ]]; then
    echo "::endgroup::"
    echo "::group::$1"
  else
    echo "=== $1"
  fi
}

run_gdb() {
  # gdb is a system binary, so it must not load the pixi libraries or Python.
  timeout 180 env -u LD_LIBRARY_PATH -u PYTHONHOME -u PYTHONPATH /usr/bin/gdb -q -batch \
    -ex "set debuginfod enabled off" \
    -ex "set pagination off" \
    -ex "set environment LD_LIBRARY_PATH ${LD_LIBRARY_PATH:-}" \
    -ex run \
    -ex "bt 40" \
    -ex "info registers rip" \
    -ex "x/i \$pc" \
    -ex "info sharedlibrary" \
    --args "$1" --gtest_brief=1
}

if [[ "${1:-}" != "--in-env" ]]; then
  [[ -n "${GITHUB_ACTIONS:-}" ]] && echo "::group::start"
  section "System"
  grep PRETTY_NAME /etc/os-release
  uname -a
  lscpu | grep -E "^(Model name|CPU\(s\)|Flags)"
  /usr/bin/c++ --version | head -n 1
  ldd --version | head -n 1
  /usr/bin/c++ -Q --help=target | grep -E "^ +-m(arch|tune)="
  /usr/bin/c++ -dM -E -x c++ /dev/null | grep -E "_FORTIFY_SOURCE|_GLIBCXX_ASSERTIONS|__STDCPP_DEFAULT_NEW_ALIGNMENT__"

  section "Installed system packages"
  dpkg-query -W -f='${Package} ${Version}\n'

  pixi run -e "$1" bash "$0" --in-env
  status=$?
  [[ -n "${GITHUB_ACTIONS:-}" ]] && echo "::endgroup::"
  exit "$status"
fi

section "Environment"
env | grep -E "^(CC|CXX|CFLAGS|CXXFLAGS|CPPFLAGS|LDFLAGS|LD_LIBRARY_PATH|LD_PRELOAD|LIBRARY_PATH|MALLOC_|GLIBC_TUNABLES)" | sort

section "Resolved libraries"
ldd build/roboplan_core/test/test_joints

for test in "${TESTS[@]}"; do
  section "gdb: ${test}"
  run_gdb "build/roboplan_core/test/${test}"
done

section "AddressSanitizer build"
# Eigen changes its allocator under ASan, which would mismatch prebuilt Pinocchio.
colcon build \
  --packages-up-to roboplan_core \
  --build-base build_asan \
  --install-base install_asan \
  --cmake-args \
  -DBUILD_PYTHON_BINDINGS=OFF \
  "-DCMAKE_CXX_FLAGS=-fuse-ld=mold -fsanitize=address -fno-omit-frame-pointer -DEIGEN_MALLOC_ALREADY_ALIGNED=1" ||
  exit 1

for test in "${TESTS[@]}"; do
  section "AddressSanitizer: ${test}"
  ASAN_OPTIONS=detect_leaks=0:symbolize=1 timeout 300 \
    "build_asan/roboplan_core/test/${test}" --gtest_brief=1
  echo "exit code: $?"
done
