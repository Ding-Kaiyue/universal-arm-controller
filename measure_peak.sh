#!/usr/bin/env bash
set -euo pipefail

# ===============================
# Measure peak memory usage for colcon build
# and provide recommended RAM + swap for users
# ===============================

WORKSPACE_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$WORKSPACE_DIR"

# Default: all packages
PACKAGE_ARG=""
while [[ $# -gt 0 ]]; do
  case "$1" in
    --package)
      shift
      PACKAGE_ARG="$1"
      shift
      ;;
    *)
      echo "Unknown argument: $1"
      exit 1
      ;;
  esac
done

# Build arguments (safe mode)
BUILD_ARGS=(
  "--executor" "sequential"
  "--parallel-workers" "1"
  "--cmake-args"
  "-DCMAKE_BUILD_TYPE=Release"
  "-DCMAKE_BUILD_PARALLEL_LEVEL=1"
  "-DCMAKE_INTERPROCEDURAL_OPTIMIZATION=OFF"
  "-DCMAKE_EXE_LINKER_FLAGS=-fuse-ld=lld"
  "-DCMAKE_SHARED_LINKER_FLAGS=-fuse-ld=lld"
)

if [[ -n "$PACKAGE_ARG" ]]; then
  BUILD_ARGS+=(--packages-select "$PACKAGE_ARG")
fi

# Ensure /usr/bin/time exists
if ! command -v /usr/bin/time >/dev/null 2>&1; then
  echo "[ERROR] /usr/bin/time not found. Install with: sudo apt install time"
  exit 1
fi

echo "=== Measuring peak memory usage ==="
echo "Safe mode: sequential, single-threaded, no LTO"
echo ""

MAX_PEAK_KB=0
MAX_PACKAGE=""

measure_pkg() {
  local pkg="$1"
  echo "Building package: $pkg ..."
  TMP_LOG=$(mktemp)

  /usr/bin/time -v colcon build --packages-select "$pkg" "${BUILD_ARGS[@]}" 1>/dev/null 2>"$TMP_LOG" || true

  PEAK_KB=$(grep "Maximum resident set size" "$TMP_LOG" | awk '{print $6}')
  rm -f "$TMP_LOG"

  PEAK_GB=$(awk "BEGIN {printf \"%.2f\", $PEAK_KB/1024/1024}")
  echo "Package '$pkg' peak RAM: ${PEAK_GB} GB"

  if (( PEAK_KB > MAX_PEAK_KB )); then
    MAX_PEAK_KB=$PEAK_KB
    MAX_PACKAGE=$pkg
  fi
}

if [[ -n "$PACKAGE_ARG" ]]; then
  measure_pkg "$PACKAGE_ARG"
else
  PKGS=$(colcon list | awk '{print $1}')
  for pkg in $PKGS; do
    measure_pkg "$pkg"
  done
fi

MAX_PEAK_GB=$(awk "BEGIN {printf \"%.2f\", $MAX_PEAK_KB/1024/1024}")
echo ""
echo "=== Summary ==="
echo "Peak RAM usage: ${MAX_PEAK_GB} GB (package: $MAX_PACKAGE)"

# ===============================
# Recommend RAM + swap for users
# ===============================
# Recommendation strategy:
# - RAM >= 1.2 × peak
# - Remaining = swap
# - Minimum swap 8 GB

RECOMMENDED_RAM_GB=$(awk "BEGIN {printf \"%.0f\", $MAX_PEAK_GB*1.2}")
RECOMMENDED_SWAP_GB=$(awk "BEGIN {s=($MAX_PEAK_GB*1.5)-$RECOMMENDED_RAM_GB; if (s<8) s=8; printf \"%.0f\", s}")

echo ""
echo "=== Recommended memory configuration for users ==="
echo "RAM:  ${RECOMMENDED_RAM_GB} GB"
echo "SWAP: ${RECOMMENDED_SWAP_GB} GB"
echo "Total: $((RECOMMENDED_RAM_GB + RECOMMENDED_SWAP_GB)) GB"
echo ""
echo "Notes:"
echo "- Safe build mode assumes sequential compilation with no LTO."
echo "- Developer build (--dev) may require more memory."
echo "- Ensure swap is enabled if RAM < recommended RAM."
