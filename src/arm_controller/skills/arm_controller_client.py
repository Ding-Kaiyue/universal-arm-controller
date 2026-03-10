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

    def execute_movel(
        self,
        x: float, y: float, z: float,
        qx: float, qy: float, qz: float, qw: float,
        mapping: str = "left_arm"
    ) -> Tuple[bool, Dict]:
        """
        执行 MoveL（直线）命令

        Args:
            x, y, z: 笛卡尔坐标位置
            qx, qy, qz, qw: 目标四元数（四元数）
            mapping: 机械臂映射名称

        Returns:
            (success: bool, response: dict)
        """
        payload = {
            "x": x,
            "y": y,
            "z": z,
            "qx": qx,
            "qy": qy,
            "qz": qz,
            "qw": qw,
            "mapping": mapping
        }

        try:
            response = requests.post(
                f"{self.server_url}/movel",
                json=payload,
                timeout=self.timeout
            )
            response.raise_for_status()
            data = response.json()
            success = data.get("status") == "success"
            return success, data
        except requests.RequestException as e:
            logger.error(f"MoveL command failed: {e}")
            return False, {"error": str(e)}

    def execute_movec(
        self,
        waypoints: List[float],
        mapping: str = "left_arm"
    ) -> Tuple[bool, Dict]:
        """
        执行 MoveC（圆形）命令

        Args:
            waypoints: 路径点列表（扁平化的笛卡尔坐标和四元数）
                      例如: [x1, y1, z1, qx1, qy1, qz1, qw1, x2, y2, z2, qx2, qy2, qz2, qw2]
            mapping: 机械臂映射名称

        Returns:
            (success: bool, response: dict)
        """
        payload = {
            "waypoints": waypoints,
            "mapping": mapping
        }

        try:
            response = requests.post(
                f"{self.server_url}/movec",
                json=payload,
                timeout=self.timeout
            )
            response.raise_for_status()
            data = response.json()
            success = data.get("status") == "success"
            return success, data
        except requests.RequestException as e:
            logger.error(f"MoveC command failed: {e}")
            return False, {"error": str(e)}

    def execute_joint_velocity(
        self,
        joint_velocities: List[float],
        mapping: str = "left_arm"
    ) -> Tuple[bool, Dict]:
        """
        执行 JointVelocity（关节速度）命令

        Args:
            joint_velocities: 6个关节速度列表（弧度/秒）
                             例如: [v1, v2, v3, v4, v5, v6]
            mapping: 机械臂映射名称

        Returns:
            (success: bool, response: dict)
        """
        payload = {
            "joint_velocities": joint_velocities,
            "mapping": mapping
        }

        try:
            response = requests.post(
                f"{self.server_url}/joint_velocity",
                json=payload,
                timeout=self.timeout
            )
            response.raise_for_status()
            data = response.json()
            success = data.get("status") == "success"
            return success, data
        except requests.RequestException as e:
            logger.error(f"JointVelocity command failed: {e}")
            return False, {"error": str(e)}

    def execute_cartesian_velocity(
        self,
        cartesian_velocities: List[float],
        mapping: str = "left_arm"
    ) -> Tuple[bool, Dict]:
        """
        执行 CartesianVelocity（笛卡尔速度）命令

        Args:
            cartesian_velocities: 6维笛卡尔速度列表
                                 例如: [vx, vy, vz, wx, wy, wz] (米/秒 和 弧度/秒)
            mapping: 机械臂映射名称

        Returns:
            (success: bool, response: dict)
        """
        payload = {
            "cartesian_velocities": cartesian_velocities,
            "mapping": mapping
        }

        try:
            response = requests.post(
                f"{self.server_url}/cartesian_velocity",
                json=payload,
                timeout=self.timeout
            )
            response.raise_for_status()
            data = response.json()
            success = data.get("status") == "success"
            return success, data
        except requests.RequestException as e:
            logger.error(f"CartesianVelocity command failed: {e}")
            return False, {"error": str(e)}


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

    def handle_movel_command(
        self,
        x: float, y: float, z: float,
        qx: float, qy: float, qz: float, qw: float,
        mapping: str = "left_arm"
    ) -> Dict:
        """处理 MoveL 命令"""
        success, response = self.client.execute_movel(x, y, z, qx, qy, qz, qw, mapping)

        if success:
            logger.info(f"MoveL executed successfully: {response}")
            return {
                "status": "success",
                "message": "MoveL command executed",
                "data": response
            }
        else:
            logger.error(f"MoveL execution failed: {response}")
            return {
                "status": "error",
                "message": "MoveL command failed",
                "error": response.get("error", "Unknown error")
            }

    def handle_movec_command(
        self,
        waypoints: List[float],
        mapping: str = "left_arm"
    ) -> Dict:
        """处理 MoveC 命令"""
        success, response = self.client.execute_movec(waypoints, mapping)

        if success:
            logger.info(f"MoveC executed successfully: {response}")
            return {
                "status": "success",
                "message": "MoveC command executed",
                "data": response
            }
        else:
            logger.error(f"MoveC execution failed: {response}")
            return {
                "status": "error",
                "message": "MoveC command failed",
                "error": response.get("error", "Unknown error")
            }

    def handle_joint_velocity_command(
        self,
        joint_velocities: List[float],
        mapping: str = "left_arm"
    ) -> Dict:
        """处理 JointVelocity 命令"""
        success, response = self.client.execute_joint_velocity(joint_velocities, mapping)

        if success:
            logger.info(f"JointVelocity executed successfully: {response}")
            return {
                "status": "success",
                "message": "JointVelocity command executed",
                "data": response
            }
        else:
            logger.error(f"JointVelocity execution failed: {response}")
            return {
                "status": "error",
                "message": "JointVelocity command failed",
                "error": response.get("error", "Unknown error")
            }

    def handle_cartesian_velocity_command(
        self,
        cartesian_velocities: List[float],
        mapping: str = "left_arm"
    ) -> Dict:
        """处理 CartesianVelocity 命令"""
        success, response = self.client.execute_cartesian_velocity(cartesian_velocities, mapping)

        if success:
            logger.info(f"CartesianVelocity executed successfully: {response}")
            return {
                "status": "success",
                "message": "CartesianVelocity command executed",
                "data": response
            }
        else:
            logger.error(f"CartesianVelocity execution failed: {response}")
            return {
                "status": "error",
                "message": "CartesianVelocity command failed",
                "error": response.get("error", "Unknown error")
            }

    def handle_nlp_intent(self, intent: str, entities: Dict) -> Dict:
        """
        处理从自然语言识别出的意图

        Args:
            intent: 识别出的意图，例如 'move_to_position', 'move_linear', 'move_arc'
            entities: 识别出的实体，例如 {'positions': [1.0, 2.0, ...], 'mapping': 'single_arm'}

        Returns:
            执行结果字典
        """
        mapping = entities.get("mapping", "left_arm")

        if intent == "move_to_position":
            # MoveJ: 关节空间中的点到点运动
            positions = entities.get("positions", [])
            if not positions:
                return {
                    "status": "error",
                    "message": "No positions provided"
                }
            return self.handle_movej_command(positions, mapping)

        elif intent == "move_linear":
            # MoveL: 笛卡尔空间中的直线运动
            x = entities.get("x")
            y = entities.get("y")
            z = entities.get("z")
            qx = entities.get("qx", 0.0)
            qy = entities.get("qy", 0.0)
            qz = entities.get("qz", 0.0)
            qw = entities.get("qw", 1.0)

            if x is None or y is None or z is None:
                return {
                    "status": "error",
                    "message": "Missing position coordinates (x, y, z)"
                }

            return self.handle_movel_command(x, y, z, qx, qy, qz, qw, mapping)

        elif intent == "move_arc":
            # MoveC: 圆形运动
            waypoints = entities.get("waypoints", [])
            if not waypoints:
                return {
                    "status": "error",
                    "message": "No waypoints provided"
                }
            return self.handle_movec_command(waypoints, mapping)

        elif intent == "joint_velocity":
            # JointVelocity: 关节速度控制
            joint_velocities = entities.get("joint_velocities", [])
            if not joint_velocities:
                return {
                    "status": "error",
                    "message": "No joint velocities provided"
                }
            return self.handle_joint_velocity_command(joint_velocities, mapping)

        elif intent == "cartesian_velocity":
            # CartesianVelocity: 笛卡尔速度控制
            cartesian_velocities = entities.get("cartesian_velocities", [])
            if not cartesian_velocities:
                return {
                    "status": "error",
                    "message": "No cartesian velocities provided"
                }
            return self.handle_cartesian_velocity_command(cartesian_velocities, mapping)

        elif intent == "move_to_home":
            success, response = self.client.movej_home(mapping)
            return {
                "status": "success" if success else "error",
                "message": f"Move to home {'successful' if success else 'failed'}",
                "data": response
            }

        elif intent == "move_to_zero":
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
        print(f"✅ MoveJ 命令已发送: {response}")
    else:
        print(f"❌ MoveJ 命令失败: {response}")

    # 执行 MoveL 命令（直线运动）
    print("\n--- MoveL 示例 ---")
    success, response = client.execute_movel(
        x=0.5, y=0.3, z=0.2,
        qx=0.0, qy=0.0, qz=0.0, qw=1.0,
        mapping="left_arm"
    )
    if success:
        print(f"✅ MoveL 命令已发送: {response}")
    else:
        print(f"❌ MoveL 命令失败: {response}")

    # 执行 JointVelocity 命令（关节速度控制）
    print("\n--- JointVelocity 示例 ---")
    joint_velocities = [0.5, 0.3, -0.2, 0.0, 0.1, 0.0]  # 弧度/秒
    success, response = client.execute_joint_velocity(joint_velocities, "left_arm")
    if success:
        print(f"✅ JointVelocity 命令已发送: {response}")
    else:
        print(f"❌ JointVelocity 命令失败: {response}")

    # 执行 CartesianVelocity 命令（笛卡尔速度控制）
    print("\n--- CartesianVelocity 示例 ---")
    cartesian_velocities = [0.1, 0.0, 0.0, 0.0, 0.0, 0.5]  # m/s 和 rad/s
    success, response = client.execute_cartesian_velocity(cartesian_velocities, "left_arm")
    if success:
        print(f"✅ CartesianVelocity 命令已发送: {response}")
    else:
        print(f"❌ CartesianVelocity 命令失败: {response}")


def example_skill_handler():
    """示例 2：通过 Skill Handler 调用各种模式"""
    print("\n=== 示例 2: 通过 Skill Handler 调用各种模式 ===\n")

    handler = OpenClawSkillHandler()

    # 1. MoveJ: 关节空间点到点运动
    print("--- 1. MoveJ (关节空间点到点) ---")
    result = handler.handle_nlp_intent(
        intent="move_to_position",
        entities={
            "positions": [0.0, -0.5, 1.0, 0.0, 0.5, 0.0],
            "mapping": "left_arm"
        }
    )
    print(f"结果: {json.dumps(result, indent=2, ensure_ascii=False)}")

    # 2. MoveL: 笛卡尔空间直线运动
    print("\n--- 2. MoveL (笛卡尔空间直线运动) ---")
    result = handler.handle_nlp_intent(
        intent="move_linear",
        entities={
            "x": 0.5,
            "y": 0.3,
            "z": 0.4,
            "qx": 0.0,
            "qy": 0.0,
            "qz": 0.0,
            "qw": 1.0,
            "mapping": "left_arm"
        }
    )
    print(f"结果: {json.dumps(result, indent=2, ensure_ascii=False)}")

    # 3. MoveC: 圆形运动
    print("\n--- 3. MoveC (圆形运动) ---")
    result = handler.handle_nlp_intent(
        intent="move_arc",
        entities={
            "waypoints": [
                0.5, 0.3, 0.4, 0.0, 0.0, 0.0, 1.0,  # 点 1: (x,y,z,qx,qy,qz,qw)
                0.5, 0.4, 0.4, 0.0, 0.0, 0.0, 1.0,  # 点 2
                0.6, 0.3, 0.4, 0.0, 0.0, 0.0, 1.0   # 点 3
            ],
            "mapping": "left_arm"
        }
    )
    print(f"结果: {json.dumps(result, indent=2, ensure_ascii=False)}")

    # 4. JointVelocity: 关节速度控制
    print("\n--- 4. JointVelocity (关节速度控制) ---")
    result = handler.handle_nlp_intent(
        intent="joint_velocity",
        entities={
            "joint_velocities": [0.5, 0.3, -0.2, 0.0, 0.1, 0.0],
            "mapping": "left_arm"
        }
    )
    print(f"结果: {json.dumps(result, indent=2, ensure_ascii=False)}")

    # 5. CartesianVelocity: 笛卡尔速度控制
    print("\n--- 5. CartesianVelocity (笛卡尔速度控制) ---")
    result = handler.handle_nlp_intent(
        intent="cartesian_velocity",
        entities={
            "cartesian_velocities": [0.1, 0.0, 0.0, 0.0, 0.0, 0.5],  # vx, vy, vz, wx, wy, wz
            "mapping": "left_arm"
        }
    )
    print(f"结果: {json.dumps(result, indent=2, ensure_ascii=False)}")


def example_openclaw_integration():
    """示例 3：如何在 OpenClaw 中集成所有模式"""
    print("\n=== 示例 3: OpenClaw 集成指南 ===\n")

    instructions = """
在 OpenClaw 中集成机械臂多模式控制 Skill 的步骤：

1. 将 arm_movej_skill.json 复制到 OpenClaw skills 目录

2. 在 OpenClaw 配置中注册 Skill：
   {
     "skills": [
       {
         "name": "arm_controller",
         "config": "arm_movej_skill.json",
         "handler": "arm_controller_client:OpenClawSkillHandler"
       }
     ]
   }

3. 在 OpenClaw 的 NLP 处理器中添加关键词识别：

   【关节空间运动 (MoveJ)】
   - "移动到 [关节位置值]" → move_to_position
   - "运动到 [关节位置值]" → move_to_position
   - 参数: positions=[j1, j2, j3, j4, j5, j6], mapping=arm

   【笛卡尔直线运动 (MoveL)】
   - "直线运动到 [位置]" → move_linear
   - "笛卡尔运动到 [x, y, z]" → move_linear
   - 参数: x, y, z, qx, qy, qz, qw, mapping

   【圆形运动 (MoveC)】
   - "沿着圆弧运动" → move_arc
   - "圆形运动经过 [多个点]" → move_arc
   - 参数: waypoints=[x1,y1,z1,qx1,qy1,qz1,qw1, ...], mapping

   【关节速度控制 (JointVelocity)】
   - "以关节速度 [速度值] 运动" → joint_velocity
   - "关节速度模式，速度为 [v1,v2,v3,v4,v5,v6]" → joint_velocity
   - 参数: joint_velocities=[v1, v2, v3, v4, v5, v6] (rad/s), mapping

   【笛卡尔速度控制 (CartesianVelocity)】
   - "笛卡尔速度运动" → cartesian_velocity
   - "速度控制，沿 X 轴 [速度]" → cartesian_velocity
   - 参数: cartesian_velocities=[vx, vy, vz, wx, wy, wz] (m/s, rad/s), mapping

   【预定义位置】
   - "运动到回家位置" → move_to_home
   - "运动到零位置" → move_to_zero
   - 参数: mapping=arm

4. OpenClaw 在识别用户意图后，会调用相应的 Skill handler

示例用户对话：

【场景 1：点到点运动】
  用户: "让左臂运动到位置 1, 0.5, -1, 0, 1, 0"
  流程: intent="move_to_position" → handle_movej_command() → /movej API

【场景 2：直线运动】
  用户: "左臂直线运动到坐标 (0.5, 0.3, 0.4)"
  流程: intent="move_linear" → handle_movel_command() → /movel API

【场景 3：圆形运动】
  用户: "让机械臂沿着圆弧运动"
  流程: intent="move_arc" → handle_movec_command() → /movec API

【场景 4：关节速度控制】
  用户: "以关节速度 0.5, 0.3, -0.2, 0, 0.1, 0 运动"
  流程: intent="joint_velocity" → handle_joint_velocity_command() → /joint_velocity API

【场景 5：笛卡尔速度控制】
  用户: "沿 X 轴以 0.1 米/秒的速度运动"
  流程: intent="cartesian_velocity" → handle_cartesian_velocity_command() → /cartesian_velocity API

【HTTP API 端点】
- POST /movej           - 关节空间点到点运动
- POST /movel           - 笛卡尔空间直线运动
- POST /movec           - 圆形运动
- POST /joint_velocity  - 关节速度控制
- POST /cartesian_velocity - 笛卡尔速度控制
- GET  /health         - 服务器健康检查

【请求格式示例】

MoveJ:
  {
    "positions": [1.57, 0, 1.57, 0, 1.57, 0],
    "mapping": "left_arm"
  }

MoveL:
  {
    "x": 0.5, "y": 0.3, "z": 0.4,
    "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0,
    "mapping": "left_arm"
  }

JointVelocity:
  {
    "joint_velocities": [0.5, 0.3, -0.2, 0.0, 0.1, 0.0],
    "mapping": "left_arm"
  }

CartesianVelocity:
  {
    "cartesian_velocities": [0.1, 0.0, 0.0, 0.0, 0.0, 0.5],
    "mapping": "left_arm"
  }

    """
    print(instructions)


if __name__ == "__main__":
    print("=" * 80)
    print("OpenClaw 与机械臂多模式控制器集成示例")
    print("支持模式: MoveJ, MoveL, MoveC, JointVelocity, CartesianVelocity")
    print("=" * 80)

    # 运行示例
    example_direct_api_call()
    example_skill_handler()
    example_openclaw_integration()

    print("\n" + "=" * 80)
    print("支持的控制模式:")
    print("  • MoveJ (关节空间点到点) - 6个关节位置")
    print("  • MoveL (笛卡尔空间直线) - 位置(x,y,z) + 方向(四元数)")
    print("  • MoveC (圆形运动) - 多个路径点")
    print("  • JointVelocity (关节速度) - 6个关节速度值")
    print("  • CartesianVelocity (笛卡尔速度) - 线速度(vx,vy,vz) + 角速度(wx,wy,wz)")
    print("=" * 80)
