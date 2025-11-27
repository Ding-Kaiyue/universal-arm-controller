#pragma once

#include <string>
#include <vector>
#include <memory>

namespace arm_controller {

class ArmControllerAPI {
public:
    static ArmControllerAPI& getInstance();

    // 初始化（内部启动 ROS 节点）
    bool initialize(int argc = 0, char** argv = nullptr);
    void shutdown();

    // MoveJ 控制 - 关节空间运动（模式自动切换）
    bool moveJ(const std::vector<double>& joint_positions, const std::string& mapping);

    // MoveL 控制 - 直线运动
    bool moveL(double x, double y, double z, double qx, double qy, double qz, double qw,
               const std::string& mapping);

    // MoveC 控制 - 圆弧运动
    bool moveC(const std::vector<double>& waypoints, const std::string& mapping);

    // 速度控制
    bool setJointVelocity(const std::vector<double>& velocities, const std::string& mapping);

    // 轨迹控制
    bool pauseTrajectory(const std::string& mapping);
    bool resumeTrajectory(const std::string& mapping);
    bool cancelTrajectory(const std::string& mapping);

    // 状态查询
    std::vector<double> getCurrentJointPositions(const std::string& mapping);
    bool isRobotStopped(const std::string& mapping);

    // 错误信息
    std::string getLastError() const;

private:
    ArmControllerAPI();
    ~ArmControllerAPI();

    ArmControllerAPI(const ArmControllerAPI&) = delete;
    ArmControllerAPI& operator=(const ArmControllerAPI&) = delete;

    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace arm_controller
