#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <rclcpp/rclcpp.hpp>

#include <controller_interfaces/srv/query_distance_field.hpp>

#include "algorithm/cartesian_path_planner/map/distance_field_interface.hpp"

namespace arm_controller::algorithm::cartesian_path_planner {

class CameraDriverEsdfMapClient final : public DistanceFieldInterface {
public:
    struct Config {
        std::string service_name{"/camera_driver/query_distance_field"};
        int request_timeout_ms{80};
        int startup_wait_timeout_ms{1000};
        std::size_t cache_max_entries{8192u};
    };

    CameraDriverEsdfMapClient(const Config& config,
                              rclcpp::Node::SharedPtr node);

    bool isInsideMap(const Eigen::Vector3d& p) const override;
    double getDistance(const Eigen::Vector3d& p) const override;
    Eigen::Vector3d getGradient(const Eigen::Vector3d& p) const override;
    DistanceFieldQueryResult queryDistanceAndGradient(
        const Eigen::Vector3d& p) const override;
    DistanceFieldQueryResultList queryDistanceAndGradientBatch(
        const Vector3dList& positions) const override;

    bool isServiceReady() const;
    std::size_t successfulQueries() const { return successful_queries_.load(); }
    std::size_t failedQueries() const { return failed_queries_.load(); }

private:
    Config config_;
    rclcpp::Node::SharedPtr node_;
    mutable rclcpp::Client<controller_interfaces::srv::QueryDistanceField>::SharedPtr client_;
    mutable std::mutex request_mutex_;
    mutable std::atomic<bool> logged_not_ready_{false};
    mutable std::atomic<bool> logged_timeout_{false};
    mutable std::atomic<std::size_t> successful_queries_{0u};
    mutable std::atomic<std::size_t> failed_queries_{0u};
};

}  // namespace arm_controller::algorithm::cartesian_path_planner
