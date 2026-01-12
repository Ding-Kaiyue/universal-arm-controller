#!/usr/bin/env bash
set -euo pipefail

# ===============================
# Universal Arm Controller Build Script
# ===============================
# Wrapper around colcon build that:
#  1. Checks system resources for safe build
#  2. Handles install/ directory layout conflicts
#  3. Applies safe build settings for low-memory environments
#
# Usage:
#   ./build.sh                    # Safe build (default)
#   ./build.sh --dev              # Developer build (LTO enabled)
#   ./build.sh --help             # Show help
#   ./build.sh -- [colcon args]   # Pass custom args to colcon
#
# ===============================

ROOT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$ROOT_DIR"

# ===============================
# Configuration
# ===============================
MIN_TOTAL_MEMORY_GB=12
ENABLE_LTO_DEFAULT=OFF
DEV_MODE=false
SHOW_HELP=false

# ===============================
# Helper functions
# ===============================
error() {
  echo "[ERROR] $1" >&2
  exit 1
}

info() {
  echo "[INFO] $1"
}

warn() {
  echo "[WARN] $1"
}

show_help() {
  cat << 'EOF'
Universal Arm Controller - Safe Build Script

USAGE:
  ./build.sh [OPTIONS] [-- COLCON_ARGS]

OPTIONS:
  --dev              Enable LTO and optimizations (requires 32GB+ memory)
  --help             Show this help message

EXAMPLES:
  # Standard safe build
  ./build.sh

  # Developer build with LTO
  ./build.sh --dev

  # Pass custom colcon arguments
  ./build.sh -- --packages-select arm_controller

ENVIRONMENT REQUIREMENTS:
  - Default mode: 12GB total memory (RAM + swap)
  - Developer mode: 32GB+ total memory
  - Disk space: 10GB free on root partition

MORE INFO:
  See docs/BUILD.md for detailed information
EOF
}

# ===============================
# Argument parsing
# ===============================
COLCON_ARGS=()

while [[ $# -gt 0 ]]; do
  case $1 in
    --dev)
      DEV_MODE=true
      ENABLE_LTO_DEFAULT=ON
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
      error "Unknown option: $1. Use --help for usage."
      ;;
  esac
done

# ===============================
# Environment checks
# ===============================
info "Checking build environment..."

RAM_GB=$(free -g | awk '/Mem:/ {print $2}')
SWAP_GB=$(free -g | awk '/Swap:/ {print $2}')
TOTAL_GB=$((RAM_GB + SWAP_GB))

info "System memory: RAM=${RAM_GB}GB, SWAP=${SWAP_GB}GB, Total=${TOTAL_GB}GB"

# Check minimum memory requirement
if [[ "$TOTAL_GB" -lt "$MIN_TOTAL_MEMORY_GB" ]]; then
  error "Insufficient memory for safe build.
Required: >= ${MIN_TOTAL_MEMORY_GB}GB total (RAM + swap)
Current : ${TOTAL_GB}GB total

SOLUTION: Enable swap with these commands:
  sudo fallocate -l 16G /swapfile
  sudo chmod 600 /swapfile
  sudo mkswap /swapfile
  sudo swapon /swapfile

Then add to /etc/fstab to persist:
  echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab"
fi

# Warn if developer mode needs more memory
if [[ "$DEV_MODE" == true && "$TOTAL_GB" -lt 32 ]]; then
  warn "Developer mode (LTO=ON) requires 32GB+ for optimal performance"
  warn "Current: ${TOTAL_GB}GB - build will be slower"
fi

# Check disk space
ROOT_AVAIL_GB=$(df -BG / | awk 'NR==2 {gsub("G","",$4); print $4}')
if [[ "$ROOT_AVAIL_GB" -lt 10 ]]; then
  error "Insufficient disk space. Need 10GB free, have ${ROOT_AVAIL_GB}GB"
fi

# ===============================
# Build configuration
# ===============================
BUILD_ARGS=(
  "--executor" "sequential"
  "--parallel-workers" "1"
  "--cmake-args"
  "-DCMAKE_BUILD_TYPE=Release"
  "-DCMAKE_INTERPROCEDURAL_OPTIMIZATION=${ENABLE_LTO_DEFAULT}"
)

# Use lld if available (more memory-efficient)
if command -v ld.lld >/dev/null 2>&1; then
  info "Using lld linker (memory-efficient)"
  BUILD_ARGS+=("-DCMAKE_EXE_LINKER_FLAGS=-fuse-ld=lld")
fi

# Add custom colcon arguments if provided
if [[ ${#COLCON_ARGS[@]} -gt 0 ]]; then
  BUILD_ARGS+=("${COLCON_ARGS[@]}")
fi

# ===============================
# Pre-build: handle install layout conflict
# ===============================
if [[ -d "install" ]]; then
  TMP_ERR=$(mktemp)
  set +e
  colcon build "${BUILD_ARGS[@]}" 2>"$TMP_ERR" 1>/dev/null
  RET=$?
  set -e

  if [ $RET -ne 0 ]; then
    ERR_CONTENT=$(cat "$TMP_ERR" 2>/dev/null || true)
    if echo "$ERR_CONTENT" | grep -q "install directory 'install' was created with the layout 'merged'"; then
      ts=$(date +%Y%m%d_%H%M%S)
      backup_dir="install_backup_$ts"
      warn "Detected install layout conflict. Backing up to '$backup_dir'"
      mv install "$backup_dir"
    fi
  fi
  rm -f "$TMP_ERR"
fi

# ===============================
# Build
# ===============================
info "Starting build..."
info "Mode: $([ "$DEV_MODE" = true ] && echo "Developer (LTO)" || echo "Safe (no LTO)")"
info "Memory: ${TOTAL_GB}GB total"
info ""
info "This may take 10-30 minutes. Please be patient..."
echo ""

set +e
colcon build "${BUILD_ARGS[@]}"
RET=$?
set -e

if [ $RET -eq 0 ]; then
  info ""
  info "✓ Build succeeded!"
  info ""
  info "Next steps:"
  info "  1. source install/setup.bash"
  info "  2. ros2 launch robotic_arm_bringup robotic_arm_real.launch.py"
  exit 0
else
  error "Build failed. See output above for details."
fi
