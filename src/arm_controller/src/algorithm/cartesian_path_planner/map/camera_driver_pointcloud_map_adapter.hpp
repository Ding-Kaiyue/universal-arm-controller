#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

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
        double occupancy_retention_sec{30.0};
        std::size_t max_cached_cells{200000u};
        bool accumulate_observed_bounds{true};
        int isolated_min_neighbor_count{2};
        int isolated_neighbor_radius_cells{1};
        int min_cluster_cell_count{1};
    };

    struct PlanarBaseCollisionConfig {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        Eigen::Vector3d center_in_base{Eigen::Vector3d(0.0, 0.0, 0.22)};
        Eigen::Vector3d size{Eigen::Vector3d(0.64, 0.64, 0.24)};
        double check_radius{0.17};
        bool unknown_is_free{true};
    };

    CameraDriverPointcloudMapAdapter(const Config& config,
                                     const rclcpp::Node::SharedPtr& node);

    bool isInsideMap(const Eigen::Vector3d& p) const override;
    double getDistance(const Eigen::Vector3d& p) const override;
    Eigen::Vector3d getGradient(const Eigen::Vector3d& p) const override;
    DistanceFieldQueryResult queryDistanceAndGradient(
        const Eigen::Vector3d& p) const override;
    DistanceFieldQueryResultList queryDistanceAndGradientBatch(
        const Vector3dList& positions) const override;

    int processedFrames() const { return processed_frames_.load(); }
    std::size_t activeCellCount() const;
    double voxelSize() const { return config_.voxel_size_m; }
    Vector3dList occupiedCellCenters(std::size_t max_count) const;
    bool isPlanarBaseCollisionFree(
        double x,
        double y,
        double yaw,
        const PlanarBaseCollisionConfig& base_config,
        double clearance) const;

private:
    struct CellKey {
        int x{0};
        int y{0};
        int z{0};

        bool operator==(const CellKey& other) const {
            return x == other.x && y == other.y && z == other.z;
        }
        bool operator<(const CellKey& other) const {
            if (x != other.x) {
                return x < other.x;
            }
            if (y != other.y) {
                return y < other.y;
            }
            return z < other.z;
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

    struct OccupiedCell {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        Eigen::Vector3d center{Eigen::Vector3d::Zero()};
        double last_observed_time_sec{0.0};
        std::uint64_t last_observed_frame{0u};
    };

    using OccupancyMap = std::unordered_map<
        CellKey,
        OccupiedCell,
        CellKeyHash,
        std::equal_to<CellKey>,
        Eigen::aligned_allocator<std::pair<const CellKey, OccupiedCell>>>;
    using AccumulatorMap = std::unordered_map<
        CellKey,
        CellAccum,
        CellKeyHash,
        std::equal_to<CellKey>,
        Eigen::aligned_allocator<std::pair<const CellKey, CellAccum>>>;

    void onPointcloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    CellKey toCellKey(const Eigen::Vector3d& p) const;
    bool pointWithinObservedBounds(const Eigen::Vector3d& p) const;
    DistanceFieldQueryResult queryDistanceAndGradientUnlocked(
        const Eigen::Vector3d& p) const;
    bool isSphereCollisionFreeUnlocked(
        const Eigen::Vector3d& center,
        double radius) const;
    int countOccupiedNeighbors(const CellKey& key,
                               const AccumulatorMap& accumulators) const;
    OccupancyMap filterSmallClusters(const OccupancyMap& occupied_cells) const;
    void purgeExpiredCellsUnlocked(double now_sec);
    void enforceCacheLimitUnlocked();
    void updateBoundsUnlocked();

    Config config_;
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;

    mutable std::mutex cloud_mutex_;
    OccupancyMap occupied_cells_;
    bool has_bounds_{false};
    Eigen::Vector3d min_bound_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d max_bound_{Eigen::Vector3d::Zero()};
    int search_radius_cells_{1};
    std::uint64_t processed_frame_seq_{0u};

    std::atomic<int> processed_frames_{0};
};

}  // namespace arm_controller::algorithm::cartesian_path_planner
