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
 * - 使用 Pinocchio 库基于 URDF 模型计算机械臂各关节的重力补偿力矩。
 * - 一个 URDF 对应一个完整模型
 * - 支持在同一 URDF 中注册多个 joint mapping（如 left_arm / right_arm）
 *
 * 使用 PIMPL 模式隐藏 Pinocchio 实现细节，加快编译速度。
 * mapping 的语义：一组 joint（JointGroup），而不是一个独立模型
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
     * @param urdf_path URDF 文件路径
     * @return 是否加载成功
     */
    bool loadUrdf(const std::string& urdf_path);

    /**
     * @brief 注册一个 joint mapping
     * @param mapping mapping 名称（如 left_arm / right_arm）
     * @param joint_names 该 mapping 对应的关节名（URDF 中的 joint name）
     * @return 是否成功
     */
    bool registerMapping(const std::string& mapping,
                         const std::vector<std::string>& joint_names);

    /**
     * @brief 是否存在 mapping
     */
    bool hasMapping(const std::string& mapping) const;

    /**
     * @brief 获取 mapping 的自由度
     */
    size_t getDof(const std::string& mapping) const;

    /**
     * @brief 计算某个 mapping 的重力补偿力矩
     * @param mapping mapping 名称
     * @param joint_positions mapping 对应的关节位置
     * @return 重力补偿力矩（顺序与 joint_names 一致）
     */
    std::vector<double> computeGravity(
        const std::string& mapping,
        const std::vector<double>& joint_positions);
private:
    // PIMPL: 隐藏 Pinocchio 实现细节
    class Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace dynamics
}  // namespace arm_controller

#endif  // ARM_CONTROLLER_DYNAMICS_GRAVITY_COMPENSATOR_HPP
