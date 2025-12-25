#ifndef ARM_CONTROLLER_DYNAMICS_GRAVITY_COMPENSATOR_HPP
#define ARM_CONTROLLER_DYNAMICS_GRAVITY_COMPENSATOR_HPP

#include <string>
#include <vector>
#include <memory>

namespace arm_controller {
namespace dynamics {

/**
 * @brief 重力补偿计算器
 *
 * 使用 Pinocchio 库基于 URDF 模型计算机械臂各关节的重力补偿力矩。
 * 支持多个机器人模型（通过 mapping 区分）。
 *
 * 使用 PIMPL 模式隐藏 Pinocchio 实现细节，加快编译速度。
 */
class GravityCompensator {
public:
    GravityCompensator();
    ~GravityCompensator();

    // 禁止拷贝
    GravityCompensator(const GravityCompensator&) = delete;
    GravityCompensator& operator=(const GravityCompensator&) = delete;

    /**
     * @brief 加载 URDF 模型
     * @param mapping 机器人映射名称
     * @param urdf_path URDF 文件路径
     * @return 是否加载成功
     */
    bool loadModel(const std::string& mapping, const std::string& urdf_path);

    /**
     * @brief 检查模型是否已加载
     * @param mapping 机器人映射名称
     * @return 是否已加载
     */
    bool hasModel(const std::string& mapping) const;

    /**
     * @brief 计算重力补偿力矩
     * @param mapping 机器人映射名称
     * @param joint_positions 关节位置 (弧度)
     * @return 各关节的重力补偿力矩，如果模型未加载则返回空向量
     */
    std::vector<double> computeGravityTorques(const std::string& mapping,
                                              const std::vector<double>& joint_positions);

    /**
     * @brief 获取模型的关节数量
     * @param mapping 机器人映射名称
     * @return 关节数量，如果模型未加载则返回 0
     */
    size_t getNumJoints(const std::string& mapping) const;

    /**
     * @brief 获取已加载模型的 URDF 路径
     * @param mapping 机器人映射名称
     * @return URDF 路径，如果未加载则返回空字符串
     */
    std::string getUrdfPath(const std::string& mapping) const;

private:
    // PIMPL: 隐藏 Pinocchio 实现细节
    class Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace dynamics
}  // namespace arm_controller

#endif  // ARM_CONTROLLER_DYNAMICS_GRAVITY_COMPENSATOR_HPP
