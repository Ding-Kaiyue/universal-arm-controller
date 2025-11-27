#pragma once

#include <string>
#include <vector>
#include <memory>

namespace arm_controller {

namespace ipc {
class CommandProducer;
class SharedMemoryManager;
}

// ============================================================================
// ArmControllerAPI：纯 C++ API 接口
// 单一职责：对外提供统一的 API，隐藏 IPC 实现细节
// 依赖倒置：依赖于 CommandProducer（抽象接口），而非具体的 IPC 实现
// ============================================================================
class ArmControllerAPI {
public:
    // 单例模式
    static ArmControllerAPI& getInstance();

    // 禁止拷贝
    ArmControllerAPI(const ArmControllerAPI&) = delete;
    ArmControllerAPI& operator=(const ArmControllerAPI&) = delete;

    // 初始化（内部启动 IPC）
    bool initialize(int argc = 0, char** argv = nullptr);
    void shutdown();

    // ========================================================================
    // 轨迹控制接口（统一的高层 API）
    // ========================================================================

    // MoveJ - 关节空间运动（模式自动切换）
    bool moveJ(const std::vector<double>& joint_positions,
               const std::string& mapping);

    // MoveL - 直线运动（笛卡尔空间）
    bool moveL(double x, double y, double z,
               double qx, double qy, double qz, double qw,
               const std::string& mapping);

    // MoveC - 圆弧运动（多个途径点）
    bool moveC(const std::vector<double>& waypoints,
               const std::string& mapping);

    // ========================================================================
    // 预留接口（未来实现）
    // ========================================================================

    bool setJointVelocity(const std::vector<double>& velocities,
                         const std::string& mapping);
    bool pauseTrajectory(const std::string& mapping);
    bool resumeTrajectory(const std::string& mapping);
    bool cancelTrajectory(const std::string& mapping);

    std::vector<double> getCurrentJointPositions(
        const std::string& mapping);
    bool isRobotStopped(const std::string& mapping);

    // ========================================================================
    // 错误与状态接口
    // ========================================================================

    std::string getLastError() const;

private:
    ArmControllerAPI();
    ~ArmControllerAPI();

    struct Impl;
    std::unique_ptr<Impl> impl_;

    // 内部辅助方法
    bool ensureInitialized();
};

}  // namespace arm_controller
