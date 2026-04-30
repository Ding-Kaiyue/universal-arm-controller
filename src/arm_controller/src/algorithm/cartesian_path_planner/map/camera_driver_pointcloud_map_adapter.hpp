#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

#include <Eigen/Core>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "algorithm/cartesian_path_planner/map/distance_field_interface.hpp"

namespace arm_controller::algorithm::cartesian_path_planner {

class CameraDriverPointcloudMapAdapter final : public DistanceFieldInterface {
public:
    struct Config {
        std::string pointcloud_topic{"/camera_driver/obstacle_pointcloud"};
        int pointcloud_queue_depth{2};
        double voxel_size_m{0.05};
        double max_distance_m{0.60};
        double observation_margin_m{0.15};
        int isolated_min_neighbor_count{2};
        int isolated_neighbor_radius_cells{1};
    };

    CameraDriverPointcloudMapAdapter(const Config& config,
                                     const rclcpp::Node::SharedPtr& node);

    bool isInsideMap(const Eigen::Vector3d& p) const override;
    double getDistance(const Eigen::Vector3d& p) const override;
    Eigen::Vector3d getGradient(const Eigen::Vector3d& p) const override;
    DistanceFieldQueryResult queryDistanceAndGradient(
        const Eigen::Vector3d& p) const override;
    std::vector<DistanceFieldQueryResult> queryDistanceAndGradientBatch(
        const std::vector<Eigen::Vector3d>& positions) const override;

    int processedFrames() const { return processed_frames_.load(); }
    std::size_t activeCellCount() const;

private:
    struct CellKey {
        int x{0};
        int y{0};
        int z{0};

        bool operator==(const CellKey& other) const {
            return x == other.x && y == other.y && z == other.z;
        }
    };

    struct CellKeyHash {
        std::size_t operator()(const CellKey& key) const {
            std::size_t seed = std::hash<int>{}(key.x);
            seed ^= std::hash<int>{}(key.y) + 0x9e3779b9u + (seed << 6u) + (seed >> 2u);
            seed ^= std::hash<int>{}(key.z) + 0x9e3779b9u + (seed << 6u) + (seed >> 2u);
            return seed;
        }
    };

    struct CellAccum {
        Eigen::Vector3d sum{Eigen::Vector3d::Zero()};
        int count{0};
    };

    using OccupancyMap = std::unordered_map<CellKey, Eigen::Vector3d, CellKeyHash>;
    using AccumulatorMap = std::unordered_map<CellKey, CellAccum, CellKeyHash>;

    void onPointcloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    CellKey toCellKey(const Eigen::Vector3d& p) const;
    bool pointWithinObservedBounds(const Eigen::Vector3d& p) const;
    int countOccupiedNeighbors(const CellKey& key,
                               const AccumulatorMap& accumulators) const;

    Config config_;
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;

    mutable std::mutex cloud_mutex_;
    OccupancyMap occupied_cells_;
    bool has_bounds_{false};
    Eigen::Vector3d min_bound_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d max_bound_{Eigen::Vector3d::Zero()};
    int search_radius_cells_{1};

    std::atomic<int> processed_frames_{0};
};

}  // namespace arm_controller::algorithm::cartesian_path_planner
