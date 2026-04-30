#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
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
    std::vector<DistanceFieldQueryResult> queryDistanceAndGradientBatch(
        const std::vector<Eigen::Vector3d>& positions) const override;

    bool isServiceReady() const;
    std::size_t successfulQueries() const { return successful_queries_.load(); }
    std::size_t failedQueries() const { return failed_queries_.load(); }

private:
    struct QueryCacheKey {
        std::uint64_t x_bits{0u};
        std::uint64_t y_bits{0u};
        std::uint64_t z_bits{0u};

        bool operator==(const QueryCacheKey& other) const {
            return x_bits == other.x_bits &&
                   y_bits == other.y_bits &&
                   z_bits == other.z_bits;
        }
    };

    struct QueryCacheKeyHash {
        std::size_t operator()(const QueryCacheKey& key) const {
            std::size_t seed = std::hash<std::uint64_t>{}(key.x_bits);
            seed ^= std::hash<std::uint64_t>{}(key.y_bits) + 0x9e3779b9u +
                    (seed << 6u) + (seed >> 2u);
            seed ^= std::hash<std::uint64_t>{}(key.z_bits) + 0x9e3779b9u +
                    (seed << 6u) + (seed >> 2u);
            return seed;
        }
    };

    QueryCacheKey makeCacheKey(const Eigen::Vector3d& p) const;
    bool tryGetCachedResult(
        const Eigen::Vector3d& p,
        DistanceFieldQueryResult* result) const;
    void storeCachedResult(
        const Eigen::Vector3d& p,
        const DistanceFieldQueryResult& result) const;

    Config config_;
    rclcpp::Node::SharedPtr node_;
    mutable rclcpp::Client<controller_interfaces::srv::QueryDistanceField>::SharedPtr client_;
    mutable std::mutex request_mutex_;
    mutable std::mutex cache_mutex_;
    mutable std::unordered_map<QueryCacheKey, DistanceFieldQueryResult, QueryCacheKeyHash>
        query_cache_;
    mutable std::atomic<bool> logged_not_ready_{false};
    mutable std::atomic<bool> logged_timeout_{false};
    mutable std::atomic<std::size_t> successful_queries_{0u};
    mutable std::atomic<std::size_t> failed_queries_{0u};
};

}  // namespace arm_controller::algorithm::cartesian_path_planner
