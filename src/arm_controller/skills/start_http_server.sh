#!/bin/bash
# 启动机械臂 HTTP 控制服务器

set -e

# 获取脚本所在目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# 项目根目录：skills -> arm_controller -> src -> universal_arm_controller
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"

echo "=========================================="
echo "启动机械臂 HTTP 控制服务器"
echo "=========================================="
echo ""
echo "工作空间: $PROJECT_ROOT"
echo ""

# Source ROS setup
source "$PROJECT_ROOT/install/setup.bash"

echo "✅ ROS 环境已加载"
echo ""
echo "启动 HTTP 服务器..."
echo "服务地址: http://127.0.0.1:8080"
echo ""
echo "OpenClaw 可以使用以下方式调用:"
echo ""
echo "  curl -X POST http://127.0.0.1:8080/movej \\"
echo "    -H 'Content-Type: application/json' \\"
echo "    -d '{\"positions\": [1.0, 2.0, 3.0, 4.0, 5.0, 6.0], \"mapping\": \"left_arm\"}'"
# curl -X POST http://127.0.0.1:8080/movej \
#   -H 'Content-Type: application/json' \
#   -d '{"positions": [1.57, -0.3236, -0.5854, 0.0, 0.5236, 0.0], "mapping": "left_arm"}'

echo ""
echo "或者用 Python:"
echo ""
echo "  python3 arm_movej.py '[1.0, 2.0, 3.0, 4.0, 5.0, 6.0]' left_arm"
echo ""
echo "按 Ctrl+C 停止服务器"
echo ""
echo "=========================================="
echo ""

# 运行 HTTP 服务器
ros2 run arm_controller arm_controller_http_server
