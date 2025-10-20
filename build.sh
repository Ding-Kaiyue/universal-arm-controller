#!/usr/bin/env bash
set -euo pipefail

# build.sh - small wrapper around colcon build that handles an existing
# install/ directory created with a different layout (merged) by
# backing it up and retrying a plain build. This lets users run the
# build without passing --merge-install explicitly.

ROOT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$ROOT_DIR"

ARGS=("$@")

echo "Running: colcon build ${ARGS[*]}"

# Try a normal build and capture stderr to detect layout conflict
TMP_ERR=$(mktemp)
set +e
colcon build "${ARGS[@]}" 2>"$TMP_ERR"
RET=$?
set -e
if [ $RET -eq 0 ]; then
  rm -f "$TMP_ERR"
  echo "Build succeeded."
  exit 0
fi

ERR_CONTENT=$(cat "$TMP_ERR" || true)
echo "$ERR_CONTENT" >&2

if echo "$ERR_CONTENT" | grep -q "install directory 'install' was created with the layout 'merged'"; then
  ts=$(date +%Y%m%d_%H%M%S)
  backup_dir="install_backup_$ts"
  echo "Detected install layout conflict. Backing up existing 'install/' -> '$backup_dir'"
  mv install "$backup_dir"
  echo "Retrying plain colcon build..."
  colcon build "${ARGS[@]}"
  echo "Build succeeded after backing up previous install to $backup_dir"
  rm -f "$TMP_ERR"
  exit 0
fi

echo "Layout conflict not detected and build failed. See error output above." >&2
rm -f "$TMP_ERR"
exit 1
