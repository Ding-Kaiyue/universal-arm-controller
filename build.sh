#!/usr/bin/env bash
set -euo pipefail

# ===============================
# Universal Arm Controller Build Script
# ===============================
# Official, resource-safe build entry for this project.
#
# Goals:
#  - Prevent system freeze / OOM during build
#  - Keep code & architecture untouched
#  - Make build behavior explicit and reproducible
#
# ===============================

ROOT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$ROOT_DIR"

# ===============================
# Configuration
# ===============================
MIN_TOTAL_MEMORY_GB=12
DEV_MEMORY_HINT_GB=32

ENABLE_LTO_DEFAULT=OFF
DEV_MODE=false
BUILD_PROFILE="safe"

# ===============================
# Logging helpers
# ===============================
info()  { echo "[INFO] $1"; }
warn()  { echo "[WARN] $1"; }
error() { echo "[ERROR] $1" >&2; exit 1; }

show_help() {
  cat << 'EOF'
Universal Arm Controller - Safe Build Script

USAGE:
  ./build.sh [OPTIONS] [-- COLCON_ARGS]

OPTIONS:
  --dev              Developer build (LTO enabled, higher memory peak)
  --help             Show this help message

DEFAULT BEHAVIOR:
  - Sequential build
  - No LTO
  - Memory-efficient linker flags
  - Explicit environment checks

ENVIRONMENT REQUIREMENTS:
  Safe build:
    - >= 12GB total memory (RAM + swap)
  Developer build (--dev):
    - Recommended >= 32GB total memory

EOF
}

# ===============================
# Argument parsing
# ===============================
COLCON_ARGS=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    --dev)
      DEV_MODE=true
      ENABLE_LTO_DEFAULT=ON
      BUILD_PROFILE="dev"
      info "Developer mode enabled (LTO=ON)"
      shift
      ;;
    --help)
      show_help
      exit 0
      ;;
    --)
      shift
      COLCON_ARGS=("$@")
      break
      ;;
    *)
      error "Unknown option: $1 (use --help)"
      ;;
  esac
done

# ===============================
# Environment checks
# ===============================
info "Checking build environment..."

RAM_GB=$(free -g | awk '/Mem:/  {print $2}')
SWAP_GB=$(free -g | awk '/Swap:/ {print $2}')
TOTAL_GB=$((RAM_GB + SWAP_GB))

info "System memory: RAM=${RAM_GB}GB, SWAP=${SWAP_GB}GB, Total=${TOTAL_GB}GB"

if [[ "$TOTAL_GB" -lt "$MIN_TOTAL_MEMORY_GB" ]]; then
  error "Insufficient memory for build.

Required: >= ${MIN_TOTAL_MEMORY_GB}GB total (RAM + swap)
Current : ${TOTAL_GB}GB

Suggested solution (example 16GB swap):

  sudo fallocate -l 16G /swapfile
  sudo chmod 600 /swapfile
  sudo mkswap /swapfile
  sudo swapon /swapfile
"
fi

if [[ "$DEV_MODE" == true && "$TOTAL_GB" -lt "$DEV_MEMORY_HINT_GB" ]]; then
  warn "Developer mode may cause high memory pressure."
  warn "Recommended total memory: >= ${DEV_MEMORY_HINT_GB}GB"
fi

ROOT_AVAIL_GB=$(df -BG / | awk 'NR==2 {gsub("G","",$4); print $4}')
if [[ "$ROOT_AVAIL_GB" -lt 10 ]]; then
  error "Insufficient disk space: ${ROOT_AVAIL_GB}GB available (need >= 10GB)"
fi

# ===============================
# Linker & memory control
# ===============================
# Default: memory-friendly ld behavior
LINKER_FLAGS="-Wl,--no-keep-memory"

if command -v ld.lld >/dev/null 2>&1; then
  info "Using lld linker (lower peak memory)"
  LINKER_FLAGS="-fuse-ld=lld"
fi

# ===============================
# Build arguments
# ===============================
# Preserve CMAKE_PREFIX_PATH from environment and ensure system paths are included
CMAKE_PREFIX_PATH_ARG="${CMAKE_PREFIX_PATH:-}"
if [[ -n "$CMAKE_PREFIX_PATH_ARG" ]]; then
  # Append system default paths to ensure system packages (like NLopt) are found
  CMAKE_PREFIX_PATH_ARG="/usr/local;/usr;${CMAKE_PREFIX_PATH_ARG}"
  CMAKE_PREFIX_PATH_ARG="-DCMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH_ARG}"
fi

BUILD_ARGS=(
  "--executor" "sequential"
  "--parallel-workers" "1"
  "--cmake-args"
  "-DCMAKE_BUILD_TYPE=Release"
  "-DCMAKE_BUILD_PARALLEL_LEVEL=1"
  "-DCMAKE_INTERPROCEDURAL_OPTIMIZATION=${ENABLE_LTO_DEFAULT}"
  "-DCMAKE_EXE_LINKER_FLAGS=${LINKER_FLAGS}"
  "-DCMAKE_SHARED_LINKER_FLAGS=${LINKER_FLAGS}"
)

if [[ -n "$CMAKE_PREFIX_PATH_ARG" ]]; then
  BUILD_ARGS+=("$CMAKE_PREFIX_PATH_ARG")
fi

if [[ ${#COLCON_ARGS[@]} -gt 0 ]]; then
  BUILD_ARGS+=("${COLCON_ARGS[@]}")
fi

# ===============================
# Install layout conflict handling
# ===============================
if [[ -d install ]]; then
  TMP_ERR=$(mktemp)
  set +e
  colcon build "${BUILD_ARGS[@]}" 1>/dev/null 2>"$TMP_ERR"
  RET=$?
  set -e

  if [[ $RET -ne 0 ]] && grep -q "layout 'merged'" "$TMP_ERR"; then
    ts=$(date +%Y%m%d_%H%M%S)
    backup="install_backup_${ts}"
    warn "Install layout conflict detected."
    warn "Backing up existing install/ -> ${backup}"
    mv install "$backup"
  fi
  rm -f "$TMP_ERR"
fi

# ===============================
# Build
# ===============================
info "Starting build..."
info "Build profile : ${BUILD_PROFILE}"
info "LTO           : ${ENABLE_LTO_DEFAULT}"
info "Total memory  : ${TOTAL_GB}GB"
info ""
info "This may take 10–30 minutes. Please be patient."
echo ""

set +e
colcon build "${BUILD_ARGS[@]}"
RET=$?
set -e

if [[ $RET -eq 0 ]]; then
  info ""
  info "✓ Build succeeded"
  info ""
  info "Next steps:"
  info "  source install/setup.bash"
  info "  ros2 launch robotic_arm_bringup robotic_arm_real.launch.py"
  exit 0
else
  error "Build failed. See output above."
fi
