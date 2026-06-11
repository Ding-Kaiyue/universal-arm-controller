#include "algorithm/cartesian_path_planner/map/camera_driver_pointcloud_map_adapter.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <queue>
#include <utility>
#include <unordered_map>

#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <Eigen/Geometry>

namespace arm_controller::algorithm::cartesian_path_planner {

CameraDriverPointcloudMapAdapter::CameraDriverPointcloudMapAdapter(
    const Config& config,
    const rclcpp::Node::SharedPtr& node)
    : config_(config),
      node_(node) {
    config_.voxel_size_m = std::max(1e-3, config_.voxel_size_m);
    config_.max_distance_m = std::max(config_.voxel_size_m, config_.max_distance_m);
    config_.observation_margin_m = std::max(0.0, config_.observation_margin_m);
    config_.occupancy_retention_sec = std::max(0.0, config_.occupancy_retention_sec);
    config_.max_cached_cells = std::max<std::size_t>(1u, config_.max_cached_cells);
    config_.isolated_min_neighbor_count = std::max(0, config_.isolated_min_neighbor_count);
    config_.isolated_neighbor_radius_cells = std::max(1, config_.isolated_neighbor_radius_cells);
    config_.min_cluster_cell_count = std::max(1, config_.min_cluster_cell_count);
    search_radius_cells_ = std::max(
        1,
        static_cast<int>(std::ceil(config_.max_distance_m / config_.voxel_size_m)));

    if (!node_) {
        return;
    }

    const auto cloud_qos =
        rclcpp::SensorDataQoS().keep_last(static_cast<size_t>(
            std::max(1, config_.pointcloud_queue_depth)));
    pointcloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
        config_.pointcloud_topic,
        cloud_qos,
        std::bind(
            &CameraDriverPointcloudMapAdapter::onPointcloud,
            this,
            std::placeholders::_1));

    RCLCPP_INFO(
        node_->get_logger(),
        "[ReactiveTask] camera_driver pointcloud adapter enabled: cloud_topic=%s voxel=%.3f max_dist=%.3f margin=%.3f isolated_neighbors>=%d radius=%d min_cluster_cells=%d",
        config_.pointcloud_topic.c_str(),
        config_.voxel_size_m,
        config_.max_distance_m,
        config_.observation_margin_m,
        config_.isolated_min_neighbor_count,
        config_.isolated_neighbor_radius_cells,
        config_.min_cluster_cell_count);
    RCLCPP_INFO(
        node_->get_logger(),
        "[ReactiveTask] camera_driver pointcloud memory map: retention=%.2fs max_cached_cells=%zu accumulate_bounds=%s",
        config_.occupancy_retention_sec,
        config_.max_cached_cells,
        config_.accumulate_observed_bounds ? "true" : "false");
}

void CameraDriverPointcloudMapAdapter::onPointcloud(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    if (!msg) {
        return;
    }

    AccumulatorMap accumulators;
    Eigen::Vector3d min_bound = Eigen::Vector3d::Constant(
        std::numeric_limits<double>::infinity());
    Eigen::Vector3d max_bound = Eigen::Vector3d::Constant(
        -std::numeric_limits<double>::infinity());
    bool has_point = false;

    try {
        sensor_msgs::PointCloud2ConstIterator<float> it_x(*msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> it_y(*msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> it_z(*msg, "z");

        for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z) {
            const Eigen::Vector3d point(*it_x, *it_y, *it_z);
            if (!point.allFinite()) {
                continue;
            }

            has_point = true;
            min_bound = min_bound.cwiseMin(point);
            max_bound = max_bound.cwiseMax(point);

            const CellKey key = toCellKey(point);
            CellAccum& cell = accumulators[key];
            cell.sum += point;
            ++cell.count;
        }
    } catch (const std::exception& e) {
        if (node_) {
            RCLCPP_WARN_THROTTLE(
                node_->get_logger(),
                *node_->get_clock(),
                2000,
                "[ReactiveTask] Failed to parse camera_driver pointcloud: %s",
                e.what());
        }
        return;
    }

    OccupancyMap frame_cells;
    frame_cells.reserve(accumulators.size());
    for (const auto& [key, cell] : accumulators) {
        if (cell.count <= 0) {
            continue;
        }
        if (config_.isolated_min_neighbor_count > 0) {
            const int neighbor_count = countOccupiedNeighbors(key, accumulators);
            if (neighbor_count < config_.isolated_min_neighbor_count) {
                continue;
            }
        }
        OccupiedCell occupied;
        occupied.center = cell.sum / static_cast<double>(cell.count);
        frame_cells.emplace(key, occupied);
    }

    if (config_.min_cluster_cell_count > 1) {
        frame_cells = filterSmallClusters(frame_cells);
    }

    {
        std::lock_guard<std::mutex> lock(cloud_mutex_);
        const double now_sec = node_ ? node_->now().seconds()
                                     : static_cast<double>(processed_frame_seq_);
        ++processed_frame_seq_;
        purgeExpiredCellsUnlocked(now_sec);
        for (auto& [key, cell] : frame_cells) {
            cell.last_observed_time_sec = now_sec;
            cell.last_observed_frame = processed_frame_seq_;
            occupied_cells_[key] = cell;
        }
        enforceCacheLimitUnlocked();
        if (has_point) {
            if (!has_bounds_ || !config_.accumulate_observed_bounds) {
                min_bound_ = min_bound;
                max_bound_ = max_bound;
            } else {
                min_bound_ = min_bound_.cwiseMin(min_bound);
                max_bound_ = max_bound_.cwiseMax(max_bound);
            }
            has_bounds_ = true;
        } else {
            updateBoundsUnlocked();
        }
    }
    ++processed_frames_;
}

CameraDriverPointcloudMapAdapter::CellKey
CameraDriverPointcloudMapAdapter::toCellKey(const Eigen::Vector3d& p) const {
    return CellKey{
        static_cast<int>(std::floor(p.x() / config_.voxel_size_m)),
        static_cast<int>(std::floor(p.y() / config_.voxel_size_m)),
        static_cast<int>(std::floor(p.z() / config_.voxel_size_m)),
    };
}

int CameraDriverPointcloudMapAdapter::countOccupiedNeighbors(
    const CellKey& key,
    const AccumulatorMap& accumulators) const {
    int neighbor_count = 0;
    for (int dx = -config_.isolated_neighbor_radius_cells;
         dx <= config_.isolated_neighbor_radius_cells;
         ++dx) {
        for (int dy = -config_.isolated_neighbor_radius_cells;
             dy <= config_.isolated_neighbor_radius_cells;
             ++dy) {
            for (int dz = -config_.isolated_neighbor_radius_cells;
                 dz <= config_.isolated_neighbor_radius_cells;
                 ++dz) {
                if (dx == 0 && dy == 0 && dz == 0) {
                    continue;
                }
                const CellKey neighbor_key{
                    key.x + dx,
                    key.y + dy,
                    key.z + dz,
                };
                if (accumulators.find(neighbor_key) == accumulators.end()) {
                    continue;
                }
                ++neighbor_count;
                if (neighbor_count >= config_.isolated_min_neighbor_count) {
                    return neighbor_count;
                }
            }
        }
    }
    return neighbor_count;
}

CameraDriverPointcloudMapAdapter::OccupancyMap
CameraDriverPointcloudMapAdapter::filterSmallClusters(
    const OccupancyMap& occupied_cells) const {
    if (config_.min_cluster_cell_count <= 1 ||
        occupied_cells.size() < static_cast<std::size_t>(config_.min_cluster_cell_count)) {
        return occupied_cells;
    }

    OccupancyMap filtered;
    filtered.reserve(occupied_cells.size());
    std::set<CellKey> visited;
    std::vector<CellKey> component;
    std::queue<CellKey> frontier;

    for (const auto& [seed_key, seed_center] : occupied_cells) {
        (void)seed_center;
        if (visited.find(seed_key) != visited.end()) {
            continue;
        }

        component.clear();
        frontier.push(seed_key);
        visited.insert(seed_key);
        while (!frontier.empty()) {
            const CellKey key = frontier.front();
            frontier.pop();
            component.push_back(key);

            for (int dx = -1; dx <= 1; ++dx) {
                for (int dy = -1; dy <= 1; ++dy) {
                    for (int dz = -1; dz <= 1; ++dz) {
                        if (dx == 0 && dy == 0 && dz == 0) {
                            continue;
                        }
                        const CellKey neighbor_key{key.x + dx, key.y + dy, key.z + dz};
                        if (visited.find(neighbor_key) != visited.end()) {
                            continue;
                        }
                        if (occupied_cells.find(neighbor_key) == occupied_cells.end()) {
                            continue;
                        }
                        visited.insert(neighbor_key);
                        frontier.push(neighbor_key);
                    }
                }
            }
        }

        if (component.size() < static_cast<std::size_t>(config_.min_cluster_cell_count)) {
            continue;
        }
        for (const auto& key : component) {
            const auto it = occupied_cells.find(key);
            if (it != occupied_cells.end()) {
                filtered.emplace(key, it->second);
            }
        }
    }

    return filtered;
}

bool CameraDriverPointcloudMapAdapter::pointWithinObservedBounds(
    const Eigen::Vector3d& p) const {
    if (!has_bounds_) {
        return false;
    }
    const Eigen::Vector3d margin =
        Eigen::Vector3d::Constant(config_.observation_margin_m);
    return (p.array() >= (min_bound_ - margin).array()).all() &&
           (p.array() <= (max_bound_ + margin).array()).all();
}

bool CameraDriverPointcloudMapAdapter::isInsideMap(const Eigen::Vector3d& p) const {
    return queryDistanceAndGradient(p).observed;
}

double CameraDriverPointcloudMapAdapter::getDistance(const Eigen::Vector3d& p) const {
    const DistanceFieldQueryResult query = queryDistanceAndGradient(p);
    return query.distance_valid ? query.distance : -1.0;
}

Eigen::Vector3d CameraDriverPointcloudMapAdapter::getGradient(const Eigen::Vector3d& p) const {
    const DistanceFieldQueryResult query = queryDistanceAndGradient(p);
    return query.gradient_valid ? query.gradient : Eigen::Vector3d::Zero();
}

DistanceFieldQueryResult CameraDriverPointcloudMapAdapter::queryDistanceAndGradient(
    const Eigen::Vector3d& p) const {
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    return queryDistanceAndGradientUnlocked(p);
}

DistanceFieldQueryResult
CameraDriverPointcloudMapAdapter::queryDistanceAndGradientUnlocked(
    const Eigen::Vector3d& p) const {
    DistanceFieldQueryResult result;
    if (occupied_cells_.empty() || !pointWithinObservedBounds(p)) {
        return result;
    }

    const CellKey center_key = toCellKey(p);
    double best_distance_sq = std::numeric_limits<double>::infinity();
    Eigen::Vector3d nearest_point = Eigen::Vector3d::Zero();

    for (int dx = -search_radius_cells_; dx <= search_radius_cells_; ++dx) {
        for (int dy = -search_radius_cells_; dy <= search_radius_cells_; ++dy) {
            for (int dz = -search_radius_cells_; dz <= search_radius_cells_; ++dz) {
                const CellKey key{
                    center_key.x + dx,
                    center_key.y + dy,
                    center_key.z + dz,
                };
                const auto it = occupied_cells_.find(key);
                if (it == occupied_cells_.end()) {
                    continue;
                }

                const double distance_sq = (p - it->second.center).squaredNorm();
                if (distance_sq < best_distance_sq) {
                    best_distance_sq = distance_sq;
                    nearest_point = it->second.center;
                }
            }
        }
    }

    result.observed = true;
    if (!std::isfinite(best_distance_sq)) {
        result.distance = config_.max_distance_m;
        result.distance_valid = true;
        return result;
    }

    result.distance = std::sqrt(best_distance_sq);
    result.distance_valid = std::isfinite(result.distance);
    if (result.distance > 1e-9) {
        result.gradient = (p - nearest_point) / result.distance;
        result.gradient_valid = result.gradient.allFinite();
    }
    return result;
}

DistanceFieldQueryResultList
CameraDriverPointcloudMapAdapter::queryDistanceAndGradientBatch(
    const Vector3dList& positions) const {
    DistanceFieldQueryResultList results;
    results.reserve(positions.size());
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    for (const auto& p : positions) {
        results.push_back(queryDistanceAndGradientUnlocked(p));
    }
    return results;
}

std::size_t CameraDriverPointcloudMapAdapter::activeCellCount() const {
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    return occupied_cells_.size();
}

Vector3dList CameraDriverPointcloudMapAdapter::occupiedCellCenters(
    const std::size_t max_count) const {
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    Vector3dList centers;
    centers.reserve(max_count == 0u ? occupied_cells_.size()
                                    : std::min(max_count, occupied_cells_.size()));
    for (const auto& [key, center] : occupied_cells_) {
        (void)key;
        centers.push_back(center.center);
        if (max_count > 0u && centers.size() >= max_count) {
            break;
        }
    }
    return centers;
}

bool CameraDriverPointcloudMapAdapter::isPlanarBaseCollisionFree(
    const double x,
    const double y,
    const double yaw,
    const PlanarBaseCollisionConfig& base_config,
    const double clearance) const {
    const double sx = std::max(0.01, base_config.size.x());
    const double sy = std::max(0.01, base_config.size.y());
    const double sz = std::max(0.01, base_config.size.z());
    const double radius = std::max(
        config_.voxel_size_m,
        base_config.check_radius + std::max(0.0, clearance));
    const double sample_radius = std::max(config_.voxel_size_m, base_config.check_radius);
    const double z_min = sample_radius;
    const double z_max = std::max(z_min, sz);
    const int nz = std::max(
        1,
        static_cast<int>(std::ceil((z_max - z_min) / sample_radius)) + 1);

    const Eigen::Rotation2Dd R2(yaw);
    const Eigen::Vector2d p_base(x, y);
    const Eigen::Vector2d c1(
        0.5 * sx - sample_radius, 0.5 * sy - sample_radius);
    const Eigen::Vector2d c2(
        0.5 * sx - sample_radius, -0.5 * sy + sample_radius);
    const Eigen::Vector2d c3(
        -0.5 * sx + sample_radius, -0.5 * sy + sample_radius);
    const Eigen::Vector2d c4(
        -0.5 * sx + sample_radius, 0.5 * sy - sample_radius);
    const std::array<Eigen::Vector2d, 4> corners{c1, c2, c3, c4};

    std::lock_guard<std::mutex> lock(cloud_mutex_);
    if (occupied_cells_.empty()) {
        return true;
    }

    auto checkSample = [&](const Eigen::Vector2d& local_xy, const double z) {
        const Eigen::Vector3d local =
            base_config.center_in_base + Eigen::Vector3d(local_xy.x(), local_xy.y(), z);
        const Eigen::Vector2d world_xy =
            p_base + R2 * Eigen::Vector2d(local.x(), local.y());
        const Eigen::Vector3d world(world_xy.x(), world_xy.y(), local.z());
        if (!pointWithinObservedBounds(world)) {
            return base_config.unknown_is_free;
        }
        return isSphereCollisionFreeUnlocked(world, radius);
    };

    for (int iz = 0; iz < nz; ++iz) {
        const double tz =
            nz <= 1 ? 0.0 : static_cast<double>(iz) / static_cast<double>(nz - 1);
        const double z = -0.5 * sz + z_min + tz * (z_max - z_min);
        for (int edge = 0; edge < 4; ++edge) {
            const Eigen::Vector2d from = corners[static_cast<std::size_t>(edge)];
            const Eigen::Vector2d to =
                corners[static_cast<std::size_t>((edge + 1) % 4)];
            const double length = (to - from).norm();
            const int n = std::max(
                1,
                static_cast<int>(std::ceil(length / sample_radius)));
            for (int i = 0; i <= n; ++i) {
                if (edge > 0 && i == 0) {
                    continue;
                }
                const double t = static_cast<double>(i) / static_cast<double>(n);
                if (!checkSample((1.0 - t) * from + t * to, z)) {
                    return false;
                }
            }
        }
    }
    return true;
}

bool CameraDriverPointcloudMapAdapter::isSphereCollisionFreeUnlocked(
    const Eigen::Vector3d& center,
    const double radius) const {
    if (occupied_cells_.empty()) {
        return true;
    }
    const CellKey center_key = toCellKey(center);
    const int radius_cells =
        std::max(1, static_cast<int>(std::ceil(radius / config_.voxel_size_m)));
    const double radius_sq = radius * radius;
    for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
        for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
            for (int dz = -radius_cells; dz <= radius_cells; ++dz) {
                const CellKey key{
                    center_key.x + dx,
                    center_key.y + dy,
                    center_key.z + dz,
                };
                const auto it = occupied_cells_.find(key);
                if (it == occupied_cells_.end()) {
                    continue;
                }
                if ((center - it->second.center).squaredNorm() < radius_sq) {
                    return false;
                }
            }
        }
    }
    return true;
}

void CameraDriverPointcloudMapAdapter::purgeExpiredCellsUnlocked(
    const double now_sec) {
    if (config_.occupancy_retention_sec <= 0.0) {
        occupied_cells_.clear();
        return;
    }
    for (auto it = occupied_cells_.begin(); it != occupied_cells_.end();) {
        if (now_sec - it->second.last_observed_time_sec >
            config_.occupancy_retention_sec) {
            it = occupied_cells_.erase(it);
        } else {
            ++it;
        }
    }
}

void CameraDriverPointcloudMapAdapter::enforceCacheLimitUnlocked() {
    if (occupied_cells_.size() <= config_.max_cached_cells) {
        return;
    }
    std::vector<std::pair<CellKey, OccupiedCell>> cells;
    cells.reserve(occupied_cells_.size());
    for (const auto& item : occupied_cells_) {
        cells.push_back(item);
    }
    const std::size_t keep = std::min(config_.max_cached_cells, cells.size());
    std::nth_element(
        cells.begin(),
        cells.begin() + static_cast<std::ptrdiff_t>(keep - 1u),
        cells.end(),
        [](const auto& a, const auto& b) {
            if (a.second.last_observed_time_sec != b.second.last_observed_time_sec) {
                return a.second.last_observed_time_sec >
                       b.second.last_observed_time_sec;
            }
            return a.second.last_observed_frame > b.second.last_observed_frame;
        });
    occupied_cells_.clear();
    occupied_cells_.reserve(keep);
    for (std::size_t i = 0; i < keep && i < cells.size(); ++i) {
        occupied_cells_.emplace(cells[i].first, cells[i].second);
    }
    updateBoundsUnlocked();
}

void CameraDriverPointcloudMapAdapter::updateBoundsUnlocked() {
    if (occupied_cells_.empty()) {
        has_bounds_ = false;
        min_bound_ = Eigen::Vector3d::Zero();
        max_bound_ = Eigen::Vector3d::Zero();
        return;
    }
    Eigen::Vector3d min_bound =
        Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
    Eigen::Vector3d max_bound =
        Eigen::Vector3d::Constant(-std::numeric_limits<double>::infinity());
    for (const auto& [key, cell] : occupied_cells_) {
        (void)key;
        min_bound = min_bound.cwiseMin(cell.center);
        max_bound = max_bound.cwiseMax(cell.center);
    }
    min_bound_ = min_bound;
    max_bound_ = max_bound;
    has_bounds_ = true;
}

}  // namespace arm_controller::algorithm::cartesian_path_planner
