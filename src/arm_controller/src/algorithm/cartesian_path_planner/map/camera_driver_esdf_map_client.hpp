#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <rclcpp/rclcpp.hpp>

#include <camera_driver/esdf/esdf_shm.hpp>

#include "algorithm/cartesian_path_planner/map/distance_field_interface.hpp"

namespace arm_controller::algorithm::cartesian_path_planner {

class CameraDriverEsdfMapClient final : public DistanceFieldInterface {
public:
    struct Config {
        std::string shm_name{camera_driver::EsdfShmReader::defaultShmName()};
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

    bool isMapReady() const;
    bool isServiceReady() const { return isMapReady(); }
    std::size_t successfulQueries() const { return successful_queries_.load(); }
    std::size_t failedQueries() const { return failed_queries_.load(); }
    int processedFrames() const;

private:
    struct SnapshotCache {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        bool valid{false};
        bool has_observed_bounds{false};
        std::uint64_t sequence{0u};
        int processed_frames{0};
        double resolution{0.05};
        double resolution_inv{20.0};
        Eigen::Vector3d origin{Eigen::Vector3d::Zero()};
        Eigen::Vector3d size{Eigen::Vector3d::Zero()};
        Eigen::Vector3d min_boundary{Eigen::Vector3d::Zero()};
        Eigen::Vector3d max_boundary{Eigen::Vector3d::Zero()};
        Eigen::Vector3i voxel_num{Eigen::Vector3i::Zero()};
        Eigen::Vector3i observed_min{Eigen::Vector3i::Zero()};
        Eigen::Vector3i observed_max{Eigen::Vector3i::Zero()};
        double unknown_distance{10000.0};
        std::vector<float> distances;
    };

    bool refreshSnapshotIfNeeded() const;
    DistanceFieldQueryResult queryFromSnapshot(
        const SnapshotCache& snapshot,
        const Eigen::Vector3d& p) const;
    bool isInMap(const SnapshotCache& snapshot, const Eigen::Vector3d& p) const;
    bool isWithinObservedBounds(
        const SnapshotCache& snapshot,
        const Eigen::Vector3i& index) const;
    Eigen::Vector3i posToIndex(
        const SnapshotCache& snapshot,
        const Eigen::Vector3d& p) const;
    Eigen::Vector3d indexToPos(
        const SnapshotCache& snapshot,
        const Eigen::Vector3i& index) const;
    int toAddress(
        const SnapshotCache& snapshot,
        const Eigen::Vector3i& index) const;

    Config config_;
    rclcpp::Node::SharedPtr node_;
    mutable camera_driver::EsdfShmReader shm_reader_;
    mutable SnapshotCache snapshot_;
    mutable std::mutex snapshot_mutex_;
    mutable std::atomic<bool> logged_not_ready_{false};
    mutable std::atomic<std::size_t> successful_queries_{0u};
    mutable std::atomic<std::size_t> failed_queries_{0u};
};

}  // namespace arm_controller::algorithm::cartesian_path_planner
