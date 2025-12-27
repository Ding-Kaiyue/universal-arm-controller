#ifndef __TRAJECTORY_SMOOTHER_HPP__
#define __TRAJECTORY_SMOOTHER_HPP__

#include <vector>
#include <memory>
#include <trajectory_interpolator/moveit_spline_adapter.hpp>
#include <rclcpp/rclcpp.hpp>

/**
 * @brief 轨迹平滑策略基类
 */
class TrajectorySmootherStrategy {
public:
    virtual ~TrajectorySmootherStrategy() = default;

    /**
     * @brief 平滑轨迹数据
     */
    virtual trajectory_interpolator::Trajectory smooth(
        const std::vector<double>& times,
        const std::vector<std::vector<double>>& positions,
        const std::vector<std::vector<double>>& velocities,
        const std::vector<std::vector<double>>& efforts,
        const std::vector<std::string>& joint_names) = 0;

    /**
     * @brief 获取策略名称
     */
    virtual const char* name() const = 0;
};

/**
 * @brief CSAPS平滑实现
 */
class CSAPSSmoother : public TrajectorySmootherStrategy {
public:
    explicit CSAPSSmoother(rclcpp::Node::SharedPtr node);

    trajectory_interpolator::Trajectory smooth(
        const std::vector<double>& times,
        const std::vector<std::vector<double>>& positions,
        const std::vector<std::vector<double>>& velocities,
        const std::vector<std::vector<double>>& efforts,
        const std::vector<std::string>& joint_names) override;

    const char* name() const override { return "CSAPS"; }

    /**
     * @brief 设置平滑因子（0.0-1.0）
     */
    void set_smoothing_factor(double factor) { smoothing_factor_ = factor; }

private:
    rclcpp::Node::SharedPtr node_;
    double smoothing_factor_ = 0.9;  // 默认值：更接近原始数据
};

/**
 * @brief 无平滑实现（直接使用原始数据）
 */
class RawSmoother : public TrajectorySmootherStrategy {
public:
    explicit RawSmoother(rclcpp::Node::SharedPtr node);

    trajectory_interpolator::Trajectory smooth(
        const std::vector<double>& times,
        const std::vector<std::vector<double>>& positions,
        const std::vector<std::vector<double>>& velocities,
        const std::vector<std::vector<double>>& efforts,
        const std::vector<std::string>& joint_names) override;

    const char* name() const override { return "Raw"; }

private:
    rclcpp::Node::SharedPtr node_;

    /**
     * @brief 从原始数据构建轨迹点
     */
    trajectory_interpolator::TrajectoryPoint build_trajectory_point(
        size_t idx,
        const std::vector<double>& times,
        const std::vector<std::vector<double>>& positions,
        const std::vector<std::vector<double>>& velocities,
        const std::vector<std::vector<double>>& efforts);
};

/**
 * @brief 轨迹平滑器（工厂和协调器）
 */
class TrajectorySmoother {
public:
    explicit TrajectorySmoother(rclcpp::Node::SharedPtr node);

    /**
     * @brief 平滑轨迹
     * @param use_csaps 是否使用CSAPS平滑，false则使用原始数据
     * @param skip_large_datasets 大数据集时是否跳过平滑（推荐true）
     */
    trajectory_interpolator::Trajectory smooth(
        const std::vector<double>& times,
        const std::vector<std::vector<double>>& positions,
        const std::vector<std::vector<double>>& velocities,
        const std::vector<std::vector<double>>& efforts,
        const std::vector<std::string>& joint_names,
        bool use_csaps = false,
        bool skip_large_datasets = true);

    /**
     * @brief 设置大数据量阈值
     */
    void set_large_dataset_threshold(size_t threshold) {
        large_dataset_threshold_ = threshold;
    }

    size_t get_large_dataset_threshold() const {
        return large_dataset_threshold_;
    }

private:
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<TrajectorySmootherStrategy> csaps_smoother_;
    std::shared_ptr<TrajectorySmootherStrategy> raw_smoother_;
    size_t large_dataset_threshold_ = 100000;  // 100k点以上视为大数据
};

#endif // __TRAJECTORY_SMOOTHER_HPP__
