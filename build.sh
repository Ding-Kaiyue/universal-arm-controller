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
# Memory requirements - empirically measured from actual builds
# Peak linker memory for this project: ~10-12GB
# Safe margin needed: +30% buffer for system overhead
MIN_TOTAL_MEMORY_GB=16
DEV_MEMORY_HINT_GB=32

ENABLE_LTO_DEFAULT=OFF
DEV_MODE=false
BUILD_PROFILE="safe"
BUILD_TARGET_PROFILE="workspace"

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
  ./build.sh [PROFILE] [OPTIONS] [-- COLCON_ARGS]

PROFILES:
  workspace          Build the whole workspace with default options (default)
  arm-full           Build only arm_controller with all controller families enabled
  motion             Build only arm_controller with motion controllers enabled
  velocity           Build only arm_controller with velocity controllers enabled
  teach              Build only arm_controller with teach controllers enabled
  motion-velocity    Build only arm_controller with motion + velocity controllers enabled
  motion-teach       Build only arm_controller with motion + teach controllers enabled
  velocity-teach     Build only arm_controller with velocity + teach controllers enabled

EXAMPLES:
  ./build.sh motion
  ./build.sh velocity
  ./build.sh arm-full --dev

OPTIONS:
  --dev              Developer build (LTO enabled, higher memory peak)
  --profile NAME     Select a build profile explicitly
  --help             Show this help message

DEFAULT BEHAVIOR:
  - Sequential build
  - No LTO
  - Memory-efficient linker flags
  - Explicit environment checks

ENVIRONMENT REQUIREMENTS:
  Safe build:
    - >= 16GB total memory (RAM + swap)
    - 10GB+ free disk space
  Developer build (--dev):
    - Recommended >= 32GB total memory

MEMORY SETUP (if insufficient):
  1. Check current memory:
     free -h

  2. Add swap (example: 16GB swap):
     sudo fallocate -l 16G /swapfile
     sudo chmod 600 /swapfile
     sudo mkswap /swapfile
     sudo swapon /swapfile
     echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab

  3. Verify:
     free -h
     swapon --show

EOF
}

# ===============================
# Argument parsing
# ===============================
COLCON_ARGS=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    workspace|arm-full|motion|velocity|teach|motion-velocity|motion-teach|velocity-teach)
      BUILD_TARGET_PROFILE="$1"
      shift
      ;;
    --profile)
      shift
      [[ $# -gt 0 ]] || error "--profile requires a value"
      case "$1" in
        workspace|arm-full|motion|velocity|teach|motion-velocity|motion-teach|velocity-teach)
          BUILD_TARGET_PROFILE="$1"
          ;;
        *)
          error "Unknown build profile: $1 (use --help)"
          ;;
      esac
      shift
      ;;
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

CRITICAL: This project requires significant peak linker memory (~10-12GB).

Required: >= ${MIN_TOTAL_MEMORY_GB}GB total (RAM + swap)
Current : ${TOTAL_GB}GB
Deficit : $((MIN_TOTAL_MEMORY_GB - TOTAL_GB))GB needed

SOLUTION:
Add swap space to reach ${MIN_TOTAL_MEMORY_GB}GB total. Example (add 16GB swap):

  sudo fallocate -l 16G /swapfile
  sudo chmod 600 /swapfile
  sudo mkswap /swapfile
  sudo swapon /swapfile
  echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab

Then verify with: free -h

After adding swap, rerun this script.
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

BASE_COLCON_ARGS=(
  "--executor" "sequential"
  "--parallel-workers" "1"
)

PROFILE_COLCON_ARGS=()
PROFILE_CMAKE_ARGS=()

case "$BUILD_TARGET_PROFILE" in
  workspace)
    ;;
  arm-full)
    PROFILE_COLCON_ARGS+=("--packages-select" "arm_controller" "--allow-overriding" "arm_controller")
    PROFILE_CMAKE_ARGS+=(
      "-DARM_CONTROLLER_BUILD_MOTION_CONTROLLERS=ON"
      "-DARM_CONTROLLER_BUILD_VELOCITY_CONTROLLERS=ON"
      "-DARM_CONTROLLER_BUILD_TEACH_CONTROLLERS=ON"
    )
    ;;
  motion)
    PROFILE_COLCON_ARGS+=("--packages-select" "arm_controller" "--allow-overriding" "arm_controller")
    PROFILE_CMAKE_ARGS+=(
      "-DCMAKE_EXPORT_COMPILE_COMMANDS=ON"
      "-DARM_CONTROLLER_BUILD_MOTION_CONTROLLERS=ON"
      "-DARM_CONTROLLER_BUILD_VELOCITY_CONTROLLERS=OFF"
      "-DARM_CONTROLLER_BUILD_TEACH_CONTROLLERS=OFF"
      "-DBUILD_PYTHON_IPC_BINDINGS=OFF"
    )
    ;;
  velocity)
    PROFILE_COLCON_ARGS+=("--packages-select" "arm_controller" "--allow-overriding" "arm_controller")
    PROFILE_CMAKE_ARGS+=(
      "-DCMAKE_EXPORT_COMPILE_COMMANDS=ON"
      "-DARM_CONTROLLER_BUILD_MOTION_CONTROLLERS=OFF"
      "-DARM_CONTROLLER_BUILD_VELOCITY_CONTROLLERS=ON"
      "-DARM_CONTROLLER_BUILD_TEACH_CONTROLLERS=OFF"
      "-DBUILD_PYTHON_IPC_BINDINGS=OFF"
    )
    ;;
  teach)
    PROFILE_COLCON_ARGS+=("--packages-select" "arm_controller" "--allow-overriding" "arm_controller")
    PROFILE_CMAKE_ARGS+=(
      "-DCMAKE_EXPORT_COMPILE_COMMANDS=ON"
      "-DARM_CONTROLLER_BUILD_MOTION_CONTROLLERS=OFF"
      "-DARM_CONTROLLER_BUILD_VELOCITY_CONTROLLERS=OFF"
      "-DARM_CONTROLLER_BUILD_TEACH_CONTROLLERS=ON"
      "-DBUILD_PYTHON_IPC_BINDINGS=OFF"
    )
    ;;
  motion-velocity)
    PROFILE_COLCON_ARGS+=("--packages-select" "arm_controller" "--allow-overriding" "arm_controller")
    PROFILE_CMAKE_ARGS+=(
      "-DCMAKE_EXPORT_COMPILE_COMMANDS=ON"
      "-DARM_CONTROLLER_BUILD_MOTION_CONTROLLERS=ON"
      "-DARM_CONTROLLER_BUILD_VELOCITY_CONTROLLERS=ON"
      "-DARM_CONTROLLER_BUILD_TEACH_CONTROLLERS=OFF"
      "-DBUILD_PYTHON_IPC_BINDINGS=OFF"
    )
    ;;
  motion-teach)
    PROFILE_COLCON_ARGS+=("--packages-select" "arm_controller" "--allow-overriding" "arm_controller")
    PROFILE_CMAKE_ARGS+=(
      "-DCMAKE_EXPORT_COMPILE_COMMANDS=ON"
      "-DARM_CONTROLLER_BUILD_MOTION_CONTROLLERS=ON"
      "-DARM_CONTROLLER_BUILD_VELOCITY_CONTROLLERS=OFF"
      "-DARM_CONTROLLER_BUILD_TEACH_CONTROLLERS=ON"
      "-DBUILD_PYTHON_IPC_BINDINGS=OFF"
    )
    ;;
  velocity-teach)
    PROFILE_COLCON_ARGS+=("--packages-select" "arm_controller" "--allow-overriding" "arm_controller")
    PROFILE_CMAKE_ARGS+=(
      "-DCMAKE_EXPORT_COMPILE_COMMANDS=ON"
      "-DARM_CONTROLLER_BUILD_MOTION_CONTROLLERS=OFF"
      "-DARM_CONTROLLER_BUILD_VELOCITY_CONTROLLERS=ON"
      "-DARM_CONTROLLER_BUILD_TEACH_CONTROLLERS=ON"
      "-DBUILD_PYTHON_IPC_BINDINGS=OFF"
    )
    ;;
  *)
    error "Unknown build profile: ${BUILD_TARGET_PROFILE}"
    ;;
esac

CMAKE_ARGS=(
  "--cmake-args"
  "-Wno-dev"
  "-DCMAKE_BUILD_TYPE=Release"
  "-DBUILD_TESTING=OFF"
  "-DARM_CONTROLLER_ENABLE_COVERAGE=OFF"
  "-DCMAKE_BUILD_PARALLEL_LEVEL=1"
  "-DCMAKE_INTERPROCEDURAL_OPTIMIZATION=${ENABLE_LTO_DEFAULT}"
  "-DCMAKE_EXE_LINKER_FLAGS=${LINKER_FLAGS}"
  "-DCMAKE_SHARED_LINKER_FLAGS=${LINKER_FLAGS}"
)

if [[ -n "$CMAKE_PREFIX_PATH_ARG" ]]; then
  CMAKE_ARGS+=("$CMAKE_PREFIX_PATH_ARG")
fi

BUILD_ARGS=(
  "${BASE_COLCON_ARGS[@]}"
  "${PROFILE_COLCON_ARGS[@]}"
)

if [[ ${#COLCON_ARGS[@]} -gt 0 ]]; then
  BUILD_ARGS+=("${COLCON_ARGS[@]}")
fi

BUILD_ARGS+=(
  "${CMAKE_ARGS[@]}"
  "${PROFILE_CMAKE_ARGS[@]}"
)

# ===============================
# Install layout conflict handling
# ===============================
if [[ -d install ]]; then
  INSTALL_LAYOUT_FILE="install/.colcon_install_layout"
  if [[ -f "$INSTALL_LAYOUT_FILE" ]] && [[ "$(cat "$INSTALL_LAYOUT_FILE")" == "merged" ]]; then
    ts=$(date +%Y%m%d_%H%M%S)
    backup="install_backup_${ts}"
    warn "Install layout conflict detected."
    warn "Backing up existing install/ -> ${backup}"
    mv install "$backup"
  fi
fi

# ===============================
# Build
# ===============================
info "Starting build..."
info "Build profile : ${BUILD_PROFILE}"
info "Target profile: ${BUILD_TARGET_PROFILE}"
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
