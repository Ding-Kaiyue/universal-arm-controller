#!/bin/bash

# ============================================================================
# test_ipc_flow.sh - 测试 IPC 完整流程的诊断脚本
# ============================================================================

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# 获取脚本所在目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
INSTALL_DIR="${PROJECT_DIR}/../../install/arm_controller"

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}IPC 流程诊断工具${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""

# 步骤 1: 检查是否有消费端进程
echo -e "${YELLOW}[步骤 1] 检查消费端进程...${NC}"
if pgrep -f "universal_arm_controller_node" > /dev/null; then
    echo -e "${GREEN}✓ arm_controller_node 正在运行${NC}"
    pgrep -af "universal_arm_controller_node" | head -1 || true
elif ros2 node list 2>/dev/null | grep -q "arm_controller"; then
    echo -e "${GREEN}✓ arm_controller ROS2 节点存在${NC}"
elif [ -f "/dev/shm/arm_controller_shm_v1" ]; then
    echo -e "${GREEN}✓ IPC 队列存在（arm_controller_node 已运行过）${NC}"
else
    echo -e "${RED}✗ 未检测到 arm_controller_node${NC}"
    echo "  建议: 启动 ROS2 系统"
    echo "  命令: ros2 launch robotic_arm_bringup robotic_arm_real.launch.py"
fi
echo ""

# 步骤 2: 运行示例
echo -e "${YELLOW}[步骤 2] 运行 example_single_arm...${NC}"
EXAMPLE_PATH=""
if command -v example_single_arm &> /dev/null; then
    EXAMPLE_PATH="example_single_arm"
elif [ -f "${INSTALL_DIR}/lib/arm_controller/example_single_arm" ]; then
    EXAMPLE_PATH="${INSTALL_DIR}/lib/arm_controller/example_single_arm"
elif [ -f "./example_single_arm" ]; then
    EXAMPLE_PATH="./example_single_arm"
fi

if [ -n "$EXAMPLE_PATH" ]; then
    "$EXAMPLE_PATH" || true
else
    echo -e "${RED}✗ 找不到 example_single_arm${NC}"
    echo "  建议: colcon build --packages-select arm_controller"
fi
echo ""

# 步骤 3: 监控 IPC 资源
echo -e "${YELLOW}[步骤 3] 检查 IPC 资源状态...${NC}"
SHM_INSPECT="${SCRIPT_DIR}/shm_inspect.sh"
if [ -f "$SHM_INSPECT" ]; then
    bash "$SHM_INSPECT"
else
    echo -e "${RED}✗ 找不到 shm_inspect.sh${NC}"
fi
echo ""

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}诊断完成${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""
echo "说明:"
echo "  1. 如果没有消费端进程，IPC 队列会在程序结束时被清理"
echo "  2. 命令'成功'意味着已推送到队列，但没有消费端会导致任务无人执行"
echo "  3. 需要 ROS2 系统运行 MoveJController 的消费线程来处理命令"
echo ""