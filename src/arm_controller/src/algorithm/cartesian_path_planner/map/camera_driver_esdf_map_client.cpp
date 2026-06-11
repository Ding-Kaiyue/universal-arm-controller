#include "algorithm/cartesian_path_planner/map/camera_driver_esdf_map_client.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace arm_controller::algorithm::cartesian_path_planner {

CameraDriverEsdfMapClient::CameraDriverEsdfMapClient(
    const Config& config,
    rclcpp::Node::SharedPtr node)
    : config_(config),
      node_(std::move(node)),
      shm_reader_(config_.shm_name) {
    if (node_) {
        RCLCPP_INFO(
            node_->get_logger(),
            "[ReactiveTask] camera_driver ESDF SHM reader enabled: shm=%s",
            config_.shm_name.c_str());
    }
}

bool CameraDriverEsdfMapClient::isInsideMap(const Eigen::Vector3d& p) const {
    const DistanceFieldQueryResult query = queryDistanceAndGradient(p);
    return query.observed && query.distance_valid;
}

double CameraDriverEsdfMapClient::getDistance(const Eigen::Vector3d& p) const {
    const DistanceFieldQueryResult query = queryDistanceAndGradient(p);
    return query.distance_valid ? query.distance : -1.0;
}

Eigen::Vector3d CameraDriverEsdfMapClient::getGradient(
    const Eigen::Vector3d& p) const {
    const DistanceFieldQueryResult query = queryDistanceAndGradient(p);
    return query.gradient_valid ? query.gradient : Eigen::Vector3d::Zero();
}

DistanceFieldQueryResult CameraDriverEsdfMapClient::queryDistanceAndGradient(
    const Eigen::Vector3d& p) const {
    Vector3dList positions;
    positions.push_back(p);
    const DistanceFieldQueryResultList results =
        queryDistanceAndGradientBatch(positions);
    if (results.empty()) {
        return DistanceFieldQueryResult{};
    }
    return results.front();
}

DistanceFieldQueryResultList
CameraDriverEsdfMapClient::queryDistanceAndGradientBatch(
    const Vector3dList& positions) const {
    DistanceFieldQueryResultList results(positions.size());
    if (positions.empty()) {
        return results;
    }

    if (!refreshSnapshotIfNeeded()) {
        if (node_ && !logged_not_ready_.exchange(true)) {
            RCLCPP_WARN(
                node_->get_logger(),
                "[ReactiveTask] camera_driver ESDF SHM snapshot is not ready: shm=%s",
                config_.shm_name.c_str());
        }
        failed_queries_.fetch_add(positions.size(), std::memory_order_relaxed);
        return results;
    }

    std::size_t ok = 0u;
    {
        std::lock_guard<std::mutex> lock(snapshot_mutex_);
        for (std::size_t i = 0; i < positions.size(); ++i) {
            results[i] = queryFromSnapshot(snapshot_, positions[i]);
            if (results[i].observed && results[i].distance_valid) {
                ++ok;
            }
        }
    }

    successful_queries_.fetch_add(ok, std::memory_order_relaxed);
    failed_queries_.fetch_add(positions.size() - ok, std::memory_order_relaxed);
    return results;
}

bool CameraDriverEsdfMapClient::refreshSnapshotIfNeeded() const {
    std::uint64_t cached_sequence = 0u;
    bool has_cached_snapshot = false;
    {
        std::lock_guard<std::mutex> lock(snapshot_mutex_);
        if (snapshot_.valid && snapshot_.has_observed_bounds) {
            cached_sequence = snapshot_.sequence;
            has_cached_snapshot = true;
        }
    }

    camera_driver::EsdfGridSnapshot shm_snapshot;
    bool updated = false;
    if (!shm_reader_.readIfNewer(cached_sequence, &shm_snapshot, &updated)) {
        return has_cached_snapshot;
    }
    if (!updated) {
        std::lock_guard<std::mutex> lock(snapshot_mutex_);
        return snapshot_.valid && snapshot_.has_observed_bounds;
    }
    if (!shm_snapshot.valid || shm_snapshot.distances.empty()) {
        return has_cached_snapshot;
    }

    SnapshotCache next;
    next.valid = shm_snapshot.valid;
    next.has_observed_bounds = shm_snapshot.has_observed_bounds;
    next.sequence = shm_snapshot.sequence;
    next.processed_frames = shm_snapshot.processed_frames;
    next.resolution = std::max(1e-6, shm_snapshot.resolution);
    next.resolution_inv = 1.0 / next.resolution;
    next.origin = shm_snapshot.origin;
    next.size = shm_snapshot.size;
    next.min_boundary = next.origin;
    next.max_boundary = next.origin + next.size;
    next.voxel_num = shm_snapshot.voxel_num;
    next.observed_min = shm_snapshot.observed_min;
    next.observed_max = shm_snapshot.observed_max;
    next.unknown_distance = shm_snapshot.unknown_distance;
    next.distances = std::move(shm_snapshot.distances);

    const std::size_t expected =
        static_cast<std::size_t>(std::max(0, next.voxel_num.x())) *
        static_cast<std::size_t>(std::max(0, next.voxel_num.y())) *
        static_cast<std::size_t>(std::max(0, next.voxel_num.z()));
    if (!next.valid || !next.has_observed_bounds || expected == 0u ||
        next.distances.size() != expected) {
        return has_cached_snapshot;
    }

    {
        std::lock_guard<std::mutex> lock(snapshot_mutex_);
        snapshot_ = std::move(next);
    }
    return true;
}

DistanceFieldQueryResult CameraDriverEsdfMapClient::queryFromSnapshot(
    const SnapshotCache& snapshot,
    const Eigen::Vector3d& p) const {
    DistanceFieldQueryResult result;
    if (!snapshot.valid || !isInMap(snapshot, p)) {
        return result;
    }

    const Eigen::Vector3d shifted =
        p - 0.5 * snapshot.resolution * Eigen::Vector3d::Ones();
    const Eigen::Vector3i base_index = posToIndex(snapshot, shifted);
    if (base_index.x() < 0 || base_index.y() < 0 || base_index.z() < 0 ||
        base_index.x() >= snapshot.voxel_num.x() - 1 ||
        base_index.y() >= snapshot.voxel_num.y() - 1 ||
        base_index.z() >= snapshot.voxel_num.z() - 1) {
        return result;
    }

    double values[2][2][2];
    for (int x = 0; x < 2; ++x) {
        for (int y = 0; y < 2; ++y) {
            for (int z = 0; z < 2; ++z) {
                const Eigen::Vector3i index =
                    base_index + Eigen::Vector3i(x, y, z);
                if (!isWithinObservedBounds(snapshot, index)) {
                    return result;
                }
                const int address = toAddress(snapshot, index);
                if (address < 0 ||
                    address >= static_cast<int>(snapshot.distances.size())) {
                    return result;
                }
                values[x][y][z] = snapshot.distances[static_cast<std::size_t>(address)];
                if (!std::isfinite(values[x][y][z])) {
                    return result;
                }
            }
        }
    }

    const Eigen::Vector3d base_position = indexToPos(snapshot, base_index);
    Eigen::Vector3d diff = (p - base_position) * snapshot.resolution_inv;
    diff = diff.cwiseMax(0.0).cwiseMin(1.0);

    const double v00 = (1.0 - diff.x()) * values[0][0][0] +
                       diff.x() * values[1][0][0];
    const double v01 = (1.0 - diff.x()) * values[0][0][1] +
                       diff.x() * values[1][0][1];
    const double v10 = (1.0 - diff.x()) * values[0][1][0] +
                       diff.x() * values[1][1][0];
    const double v11 = (1.0 - diff.x()) * values[0][1][1] +
                       diff.x() * values[1][1][1];
    const double v0 = (1.0 - diff.y()) * v00 + diff.y() * v10;
    const double v1 = (1.0 - diff.y()) * v01 + diff.y() * v11;

    result.distance = (1.0 - diff.z()) * v0 + diff.z() * v1;
    result.observed = true;
    result.distance_valid = std::isfinite(result.distance);

    result.gradient.z() = (v1 - v0) * snapshot.resolution_inv;
    result.gradient.y() =
        ((1.0 - diff.z()) * (v10 - v00) + diff.z() * (v11 - v01)) *
        snapshot.resolution_inv;
    result.gradient.x() =
        (1.0 - diff.z()) * (1.0 - diff.y()) *
            (values[1][0][0] - values[0][0][0]) +
        (1.0 - diff.z()) * diff.y() *
            (values[1][1][0] - values[0][1][0]) +
        diff.z() * (1.0 - diff.y()) *
            (values[1][0][1] - values[0][0][1]) +
        diff.z() * diff.y() *
            (values[1][1][1] - values[0][1][1]);
    result.gradient.x() *= snapshot.resolution_inv;
    result.gradient_valid = result.gradient.allFinite();

    if (result.distance >= snapshot.unknown_distance * 0.5) {
        result.gradient = Eigen::Vector3d::Zero();
        result.gradient_valid = true;
    }
    return result;
}

bool CameraDriverEsdfMapClient::isInMap(
    const SnapshotCache& snapshot,
    const Eigen::Vector3d& p) const {
    return p.x() >= snapshot.min_boundary.x() + 1e-4 &&
           p.y() >= snapshot.min_boundary.y() + 1e-4 &&
           p.z() >= snapshot.min_boundary.z() + 1e-4 &&
           p.x() <= snapshot.max_boundary.x() - 1e-4 &&
           p.y() <= snapshot.max_boundary.y() - 1e-4 &&
           p.z() <= snapshot.max_boundary.z() - 1e-4;
}

bool CameraDriverEsdfMapClient::isWithinObservedBounds(
    const SnapshotCache& snapshot,
    const Eigen::Vector3i& index) const {
    if (!snapshot.has_observed_bounds) {
        return false;
    }
    return index.x() >= snapshot.observed_min.x() &&
           index.y() >= snapshot.observed_min.y() &&
           index.z() >= snapshot.observed_min.z() &&
           index.x() <= snapshot.observed_max.x() &&
           index.y() <= snapshot.observed_max.y() &&
           index.z() <= snapshot.observed_max.z();
}

Eigen::Vector3i CameraDriverEsdfMapClient::posToIndex(
    const SnapshotCache& snapshot,
    const Eigen::Vector3d& p) const {
    Eigen::Vector3i index;
    for (int i = 0; i < 3; ++i) {
        index(i) = static_cast<int>(
            std::floor((p(i) - snapshot.origin(i)) * snapshot.resolution_inv));
    }
    return index;
}

Eigen::Vector3d CameraDriverEsdfMapClient::indexToPos(
    const SnapshotCache& snapshot,
    const Eigen::Vector3i& index) const {
    Eigen::Vector3d p;
    for (int i = 0; i < 3; ++i) {
        p(i) = (static_cast<double>(index(i)) + 0.5) *
                   snapshot.resolution +
               snapshot.origin(i);
    }
    return p;
}

int CameraDriverEsdfMapClient::toAddress(
    const SnapshotCache& snapshot,
    const Eigen::Vector3i& index) const {
    if (index.x() < 0 || index.y() < 0 || index.z() < 0 ||
        index.x() >= snapshot.voxel_num.x() ||
        index.y() >= snapshot.voxel_num.y() ||
        index.z() >= snapshot.voxel_num.z()) {
        return -1;
    }
    return index.x() * snapshot.voxel_num.y() * snapshot.voxel_num.z() +
           index.y() * snapshot.voxel_num.z() + index.z();
}

bool CameraDriverEsdfMapClient::isMapReady() const {
    if (!refreshSnapshotIfNeeded()) {
        return false;
    }
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    return snapshot_.valid && snapshot_.has_observed_bounds;
}

int CameraDriverEsdfMapClient::processedFrames() const {
    refreshSnapshotIfNeeded();
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    return snapshot_.processed_frames;
}

}  // namespace arm_controller::algorithm::cartesian_path_planner
