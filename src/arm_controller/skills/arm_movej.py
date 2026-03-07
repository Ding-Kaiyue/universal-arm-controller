#!/usr/bin/env python3
"""
OpenClaw 调用机械臂 MoveJ 的简单包装
"""

import requests
import json
import sys

def movej(positions, mapping="left_arm"):
    """
    控制机械臂执行 MoveJ

    Args:
        positions: 关节位置列表，例如 [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
        mapping: 'left_arm' 或 'right_arm'

    Returns:
        dict: 响应结果
    """
    payload = {
        "positions": positions,
        "mapping": mapping
    }

    try:
        response = requests.post(
            "http://127.0.0.1:8080/movej",
            json=payload,
            timeout=5
        )
        result = response.json()
        return result
    except Exception as e:
        return {"error": str(e), "status": "error"}


if __name__ == "__main__":
    # 测试：python arm_movej.py "[-1.57, -0.3236, -0.5854, 0.0, 0.5236, 0.0]" "left_arm"
    if len(sys.argv) < 2:
        print("Usage: arm_movej.py '<positions_json>' [mapping]")
        print("Example: arm_movej.py '[-1.57, -0.3236, -0.5854, 0.0, 0.5236, 0.0]' left_arm")
        sys.exit(1)

    positions = json.loads(sys.argv[1])
    mapping = sys.argv[2] if len(sys.argv) > 2 else "left_arm"

    result = movej(positions, mapping)
    print(json.dumps(result, indent=2, ensure_ascii=False))
