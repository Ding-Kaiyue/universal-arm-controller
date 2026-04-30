#include "algorithm/cartesian_path_planner/map/camera_driver_pointcloud_map_adapter.hpp"

#include <cmath>
#include <limits>
#include <unordered_map>

#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace arm_controller::algorithm::cartesian_path_planner {

CameraDriverPointcloudMapAdapter::CameraDriverPointcloudMapAdapter(
    const Config& config,
    const rclcpp::Node::SharedPtr& node)
    : config_(config),
      node_(node) {
    config_.voxel_size_m = std::max(1e-3, config_.voxel_size_m);
    config_.max_distance_m = std::max(config_.voxel_size_m, config_.max_distance_m);
    config_.observation_margin_m = std::max(0.0, config_.observation_margin_m);
    config_.isolated_min_neighbor_count = std::max(0, config_.isolated_min_neighbor_count);
    config_.isolated_neighbor_radius_cells = std::max(1, config_.isolated_neighbor_radius_cells);
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
        "[ReactiveTask] camera_driver pointcloud adapter enabled: cloud_topic=%s voxel=%.3f max_dist=%.3f margin=%.3f isolated_neighbors>=%d radius=%d",
        config_.pointcloud_topic.c_str(),
        config_.voxel_size_m,
        config_.max_distance_m,
        config_.observation_margin_m,
        config_.isolated_min_neighbor_count,
        config_.isolated_neighbor_radius_cells);
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

    OccupancyMap occupied_cells;
    occupied_cells.reserve(accumulators.size());
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
        occupied_cells.emplace(key, cell.sum / static_cast<double>(cell.count));
    }

    {
        std::lock_guard<std::mutex> lock(cloud_mutex_);
        occupied_cells_ = std::move(occupied_cells);
        has_bounds_ = has_point;
        if (has_point) {
            min_bound_ = min_bound;
            max_bound_ = max_bound;
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
    DistanceFieldQueryResult result;

    std::lock_guard<std::mutex> lock(cloud_mutex_);
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

                const double distance_sq = (p - it->second).squaredNorm();
                if (distance_sq < best_distance_sq) {
                    best_distance_sq = distance_sq;
                    nearest_point = it->second;
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

std::vector<DistanceFieldQueryResult>
CameraDriverPointcloudMapAdapter::queryDistanceAndGradientBatch(
    const std::vector<Eigen::Vector3d>& positions) const {
    std::vector<DistanceFieldQueryResult> results;
    results.reserve(positions.size());
    for (const auto& p : positions) {
        results.push_back(queryDistanceAndGradient(p));
    }
    return results;
}

std::size_t CameraDriverPointcloudMapAdapter::activeCellCount() const {
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    return occupied_cells_.size();
}

}  // namespace arm_controller::algorithm::cartesian_path_planner
