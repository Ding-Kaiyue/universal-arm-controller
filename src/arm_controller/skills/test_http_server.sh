#!/bin/bash
# 测试机械臂 HTTP 服务器

echo "=========================================="
echo "测试机械臂 MoveJ HTTP 服务器"
echo "=========================================="
echo ""

# 检查服务器是否运行
echo "1️⃣  检查服务器健康状态..."
response=$(curl -s http://127.0.0.1:8080/health 2>&1)

if [[ $response == *"status"* ]]; then
    echo "✅ 服务器运行正常"
    echo "   响应: $response"
else
    echo "❌ 服务器未运行或不可达"
    echo "   请先执行: bash start_http_server.sh"
    exit 1
fi

echo ""
echo "2️⃣  测试 MoveJ 命令..."

# 测试 MoveJ 左臂
result=$(curl -s -X POST http://127.0.0.1:8080/movej \
  -H 'Content-Type: application/json' \
  -d '{"positions": [-1.57, -0.3236, -0.5854, 0.0, 0.5236, 0.0], "mapping": "left_arm"}')

echo "左臂 MoveJ 响应:"
echo "$result" | python3 -m json.tool

echo ""
echo "✅ 所有测试完成！"
echo ""
echo "OpenClaw 可以通过以下方式调用:"
echo "  1. HTTP API: curl -X POST http://127.0.0.1:8080/movej ..."
echo "  2. Python:   python3 arm_movej.py '[...positions...]' mapping"
echo "  3. 配置文件: openclaw_tools.json 或 arm_movej_skill.json"
