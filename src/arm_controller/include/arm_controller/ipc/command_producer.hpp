#ifndef __COMMAND_PRODUCER_HPP__
#define __COMMAND_PRODUCER_HPP__

#include "ipc_types.hpp"
#include "shm_manager.hpp"
#include <vector>
#include <string>
#include <memory>

namespace arm_controller::ipc {

// ============================================================================
// CommandValidator：验证命令的有效性
// 单一职责：参数验证
// ============================================================================
class CommandValidator {
public:
    struct ValidationResult {
        bool valid;
        std::string error_message;
    };

    static ValidationResult validateMapping(const std::string& mapping);
    static ValidationResult validateJointPositions(
        const std::vector<double>& positions);
    static ValidationResult validateJointVelocities(
        const std::vector<double>& velocities);
    static ValidationResult validateCartesianVelocities(
        const std::vector<double>& velocities);
    static ValidationResult validatePose(
        double x, double y, double z,
        double qx, double qy, double qz, double qw);
    static ValidationResult validateWaypoints(
        const std::vector<double>& waypoints);
};

// ============================================================================
// CommandBuilder：构建轨迹命令
// 单一职责：命令对象的构造与配置
// 设计模式：Builder Pattern（避免重复代码）
// ============================================================================
class CommandBuilder {
public:
    explicit CommandBuilder(uint32_t producer_id = 0);

    CommandBuilder& withMode(const std::string& mode);
    CommandBuilder& withMapping(const std::string& mapping);
    CommandBuilder& withJointPositions(
        const std::vector<double>& positions);
    CommandBuilder& withJointVelocities(
        const std::vector<double>& velocities);
    CommandBuilder& withCartesianVelocities(
        const std::vector<double>& velocities);
    CommandBuilder& withPose(
        double x, double y, double z,
        double qx, double qy, double qz, double qw);
    CommandBuilder& withWaypoints(
        const std::vector<double>& waypoints);

    TrajectoryCommand build();

private:
    TrajectoryCommand cmd_;
    void generateCommandId();
    void updateTimestamp();
};

// ============================================================================
// CommandProducer：生产者接口
// 单一职责：命令的推送与同步
// 依赖倒置：依赖于 SharedMemoryManager（抽象），而非具体实现
// ============================================================================
class CommandProducer {
public:
    explicit CommandProducer(
        std::shared_ptr<SharedMemoryManager> shm_manager,
        uint32_t producer_id = 0);

    // 推送命令到共享队列
    bool pushCommand(const TrajectoryCommand& cmd);

    // 获取最后一条错误
    std::string getLastError() const { return last_error_; }

    // 检查是否已初始化
    bool isInitialized() const { return initialized_; }

private:
    std::shared_ptr<SharedMemoryManager> shm_manager_;
    uint32_t producer_id_;
    bool initialized_ = false;
    std::string last_error_;

    bool ensureInitialized();
};

}  // namespace arm_controller::ipc

#endif  // __COMMAND_PRODUCER_HPP__
