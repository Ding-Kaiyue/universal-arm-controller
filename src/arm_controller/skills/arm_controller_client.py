#!/usr/bin/env python3
"""
OpenClaw 与机械臂 MoveJ 控制器集成示例

这个脚本展示了如何在 OpenClaw 中定义和使用 MoveJ 控制 Skill。
"""

import requests
import json
import logging
from typing import List, Dict, Optional, Tuple

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class ArmControllerClient:
    """机械臂控制客户端"""

    def __init__(self, server_url: str = "http://127.0.0.1:8080"):
        """
        初始化客户端

        Args:
            server_url: HTTP 服务器地址
        """
        self.server_url = server_url
        self.timeout = 30

    def check_health(self) -> bool:
        """
        检查服务器健康状态

        Returns:
            True if server is healthy, False otherwise
        """
        try:
            response = requests.get(
                f"{self.server_url}/health",
                timeout=self.timeout
            )
            return response.status_code == 200
        except requests.RequestException as e:
            logger.error(f"Health check failed: {e}")
            return False

    def execute_movej(
        self,
        positions: List[float],
        mapping: str = "left_arm"
    ) -> Tuple[bool, Dict]:
        """
        执行 MoveJ 命令

        Args:
            positions: 关节目标位置列表（弧度），例如 [1.57, 0, 1.57, 0, 1.57, 0]
            mapping: 机械臂映射名称，可选值：'left_arm'、'right_arm'

        Returns:
            (success: bool, response: dict) - 命令是否成功以及响应数据
        """
        payload = {
            "positions": positions,
            "mapping": mapping
        }

        try:
            response = requests.post(
                f"{self.server_url}/movej",
                json=payload,
                timeout=self.timeout
            )
            response.raise_for_status()
            data = response.json()
            success = data.get("status") == "success"
            return success, data
        except requests.RequestException as e:
            logger.error(f"MoveJ command failed: {e}")
            return False, {"error": str(e)}

    def movej_home(self, mapping: str = "left_arm") -> Tuple[bool, Dict]:
        """移动到回家位置"""
        # 这些是示例的回家位置，根据实际机械臂配置调整
        home_positions = [-1.57, -0.3236, -0.5854, 0.0, 0.5236, 0.0]
        return self.execute_movej(home_positions, mapping)

    def movej_zero(self, mapping: str = "left_arm") -> Tuple[bool, Dict]:
        """移动到零位置"""
        zero_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        return self.execute_movej(zero_positions, mapping)


class OpenClawSkillHandler:
    """OpenClaw Skill 处理器 - 这是在 OpenClaw 中应该如何集成的示例"""

    def __init__(self):
        self.client = ArmControllerClient()

    def handle_movej_command(
        self,
        positions: List[float],
        mapping: str = "left_arm"
    ) -> Dict:
        """
        OpenClaw 将调用这个方法来执行 MoveJ 命令

        这个方法的签名应该与 Skill JSON 定义中的请求参数一致。
        """
        success, response = self.client.execute_movej(positions, mapping)

        if success:
            logger.info(f"MoveJ executed successfully: {response}")
            return {
                "status": "success",
                "message": "MoveJ command executed",
                "data": response
            }
        else:
            logger.error(f"MoveJ execution failed: {response}")
            return {
                "status": "error",
                "message": "MoveJ command failed",
                "error": response.get("error", "Unknown error")
            }

    def handle_nlp_intent(self, intent: str, entities: Dict) -> Dict:
        """
        处理从自然语言识别出的意图

        Args:
            intent: 识别出的意图，例如 'move_to_position'
            entities: 识别出的实体，例如 {'positions': [1.0, 2.0, ...], 'mapping': 'single_arm'}

        Returns:
            执行结果字典
        """
        if intent == "move_to_position":
            positions = entities.get("positions", [])
            mapping = entities.get("mapping", "single_arm")

            if not positions:
                return {
                    "status": "error",
                    "message": "No positions provided"
                }

            return self.handle_movej_command(positions, mapping)

        elif intent == "move_to_home":
            mapping = entities.get("mapping", "single_arm")
            success, response = self.client.movej_home(mapping)
            return {
                "status": "success" if success else "error",
                "message": f"Move to home {'successful' if success else 'failed'}",
                "data": response
            }

        elif intent == "move_to_zero":
            mapping = entities.get("mapping", "single_arm")
            success, response = self.client.movej_zero(mapping)
            return {
                "status": "success" if success else "error",
                "message": f"Move to zero {'successful' if success else 'failed'}",
                "data": response
            }

        else:
            return {
                "status": "error",
                "message": f"Unknown intent: {intent}"
            }


# ============================================================================
# OpenClaw 集成示例
# ============================================================================

def example_direct_api_call():
    """示例 1：直接调用 API"""
    print("\n=== 示例 1: 直接 API 调用 ===\n")

    client = ArmControllerClient()

    # 检查服务器健康状态
    if client.check_health():
        print("✅ 服务器运行正常")
    else:
        print("❌ 服务器不可用，请先启动 arm_controller_http_server")
        return

    # 执行 MoveJ 命令
    positions = [-1.57, -0.3236, -0.5854, 0.0, 0.5236, 0.0]
    success, response = client.execute_movej(positions, "left_arm")

    if success:
        print(f"✅ 命令已发送: {response}")
    else:
        print(f"❌ 命令失败: {response}")


def example_skill_handler():
    """示例 2：通过 Skill Handler 调用"""
    print("\n=== 示例 2: 通过 Skill Handler 调用 ===\n")

    handler = OpenClawSkillHandler()

    # 模拟 OpenClaw 识别出的自然语言意图
    result = handler.handle_nlp_intent(
        intent="move_to_position",
        entities={
            "positions": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            "mapping": "left_arm"
        }
    )
    print(f"结果: {json.dumps(result, indent=2, ensure_ascii=False)}")


def example_openclaw_integration():
    """示例 3：如何在 OpenClaw 中集成"""
    print("\n=== 示例 3: OpenClaw 集成指南 ===\n")

    instructions = """
在 OpenClaw 中集成机械臂控制 Skill 的步骤：

1. 将 arm_movej_skill.json 复制到 OpenClaw skills 目录

2. 在 OpenClaw 配置中注册 Skill：
   {
     "skills": [
       {
         "name": "movej_controller",
         "config": "arm_movej_skill.json",
         "handler": "arm_controller_client:OpenClawSkillHandler"
       }
     ]
   }

3. 在 OpenClaw 的 NLP 处理器中添加关键词识别：
   - "移动到 [位置值]" → execute_movej
   - "运动到 [位置值]" → execute_movej
   - "左臂/右臂" → 提取 mapping 参数

4. OpenClaw 在识别用户意图后，会调用相应的 Skill handler

示例用户对话：
  用户: "让左臂运动到位置 1, 0.5, -1, 0, 1, 0"

  OpenClaw 处理流程:
  1. NLP 识别: intent="move_to_position", entities={"positions": [...], "mapping": "left_arm"}
  2. 调用 handler.handle_nlp_intent(intent, entities)
  3. Handler 调用 client.execute_movej(positions, mapping)
  4. HTTP 请求发送到 127.0.0.1:8080/movej
  5. 返回结果给用户
    """
    print(instructions)


if __name__ == "__main__":
    print("=" * 70)
    print("OpenClaw 与机械臂 MoveJ 控制器集成示例")
    print("=" * 70)

    # 运行示例
    example_direct_api_call()
    example_skill_handler()
    example_openclaw_integration()

    print("\n" + "=" * 70)
    print("更多信息请查看 arm_movej_skill.json 和源代码注释")
    print("=" * 70)
