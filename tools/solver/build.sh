#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
emsdk_root="${EMSDK_ROOT:-}"
if [[ -z "$emsdk_root" || ! -f "$emsdk_root/emsdk_env.sh" ]]; then
  printf 'Set EMSDK_ROOT to an installed Emscripten SDK directory.\n' >&2
  exit 2
fi

source "$emsdk_root/emsdk_env.sh" >/dev/null
build_dir="${TMPDIR:-/tmp}/hoverslam-osqp-wasm-build"
emcmake cmake -S "$repo_root/tools/solver" -B "$build_dir" \
  -DCMAKE_BUILD_TYPE=Release \
  -DFETCHCONTENT_SOURCE_DIR_QDLDL="$repo_root/vendor/osqp/algebra/_common/lin_sys/qdldl/qdldl-src"
cmake --build "$build_dir" --target osqp_bindings --parallel
