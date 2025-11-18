#!/bin/bash

# 电机控制服务测试脚本
# 使用新的 MotorControl 服务代替 Disable/Enable 控制器

echo "=========================================="
echo "电机控制服务测试"
echo "=========================================="
echo ""

# 获取映射名称（默认为 single_arm）
MAPPING="${1:-single_arm}"
echo "使用映射: $MAPPING"
echo ""

# 测试1: 失能电机
echo "测试1: 失能电机（MIT 模式）"
echo "命令:"
echo "ros2 service call /motor_control controller_interfaces/srv/MotorControl '{mapping: \"$MAPPING\", action: \"Disable\", mode: 3}'"
echo ""

# 测试2: 使能电机（MIT 模式 = 0x03）
echo "测试2: 使能电机（MIT 模式）"
echo "命令:"
echo "ros2 service call /motor_control controller_interfaces/srv/MotorControl '{mapping: \"$MAPPING\", action: \"Enable\", mode: 3}'"
echo ""

# 测试3: 使能电机（速度模式 = 0x04）
echo "测试3: 使能电机（速度模式）"
echo "命令:"
echo "ros2 service call /motor_control controller_interfaces/srv/MotorControl '{mapping: \"$MAPPING\", action: \"Enable\", mode: 4}'"
echo ""

echo "=========================================="
echo "可用的电机模式:"
echo "  EFFORT_MODE         = 0x02 (2)"
echo "  MIT_MODE            = 0x03 (3)  <- 推荐使用"
echo "  SPEED_MODE          = 0x04 (4)"
echo "  POSITION_ABS_MODE   = 0x05 (5)"
echo "  POSITION_INC_MODE   = 0x06 (6)"
echo "=========================================="
