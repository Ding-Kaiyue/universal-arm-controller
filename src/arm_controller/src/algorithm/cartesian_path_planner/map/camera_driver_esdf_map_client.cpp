#include "algorithm/cartesian_path_planner/map/camera_driver_esdf_map_client.hpp"

#include <chrono>
#include <cstring>
#include <future>
#include <unordered_map>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/point.hpp>

namespace arm_controller::algorithm::cartesian_path_planner {

CameraDriverEsdfMapClient::CameraDriverEsdfMapClient(
    const Config& config,
    rclcpp::Node::SharedPtr node)
    : config_(config),
      node_(std::move(node)) {
    config_.request_timeout_ms = std::max(1, config_.request_timeout_ms);
    config_.startup_wait_timeout_ms = std::max(1, config_.startup_wait_timeout_ms);
    if (!node_) {
        return;
    }

    client_ = node_->create_client<controller_interfaces::srv::QueryDistanceField>(
        config_.service_name);
    const bool ready = client_->wait_for_service(
        std::chrono::milliseconds(config_.startup_wait_timeout_ms));
    RCLCPP_INFO(
        node_->get_logger(),
        "[ReactiveTask] camera_driver ESDF client enabled: service=%s request_timeout_ms=%d startup_wait_ms=%d ready=%s",
        config_.service_name.c_str(),
        config_.request_timeout_ms,
        config_.startup_wait_timeout_ms,
        ready ? "true" : "false");
}

bool CameraDriverEsdfMapClient::isInsideMap(const Eigen::Vector3d& p) const {
    return queryDistanceAndGradient(p).observed;
}

double CameraDriverEsdfMapClient::getDistance(const Eigen::Vector3d& p) const {
    const DistanceFieldQueryResult query = queryDistanceAndGradient(p);
    return query.distance_valid ? query.distance : -1.0;
}

Eigen::Vector3d CameraDriverEsdfMapClient::getGradient(const Eigen::Vector3d& p) const {
    const DistanceFieldQueryResult query = queryDistanceAndGradient(p);
    return query.gradient_valid ? query.gradient : Eigen::Vector3d::Zero();
}

DistanceFieldQueryResult CameraDriverEsdfMapClient::queryDistanceAndGradient(
    const Eigen::Vector3d& p) const {
    const std::vector<DistanceFieldQueryResult> results =
        queryDistanceAndGradientBatch({p});
    if (results.empty()) {
        return DistanceFieldQueryResult{};
    }
    return results.front();
}

std::vector<DistanceFieldQueryResult>
CameraDriverEsdfMapClient::queryDistanceAndGradientBatch(
    const std::vector<Eigen::Vector3d>& positions) const {
    std::vector<DistanceFieldQueryResult> results(positions.size());
    if (positions.empty()) {
        return results;
    }
    if (!node_ || !client_) {
        failed_queries_.fetch_add(positions.size(), std::memory_order_relaxed);
        return results;
    }
    if (!client_->service_is_ready()) {
        if (!logged_not_ready_.exchange(true)) {
            RCLCPP_WARN(
                node_->get_logger(),
                "[ReactiveTask] camera_driver ESDF service is not ready: %s",
                config_.service_name.c_str());
        }
        failed_queries_.fetch_add(positions.size(), std::memory_order_relaxed);
        return results;
    }

    std::vector<std::size_t> miss_indices;
    miss_indices.reserve(positions.size());
    std::unordered_map<QueryCacheKey, std::vector<std::size_t>, QueryCacheKeyHash>
        dedup_miss_indices;
    std::vector<Eigen::Vector3d> dedup_miss_positions;
    dedup_miss_positions.reserve(positions.size());

    for (std::size_t i = 0; i < positions.size(); ++i) {
        if (tryGetCachedResult(positions[i], &results[i])) {
            continue;
        }
        miss_indices.push_back(i);
        const QueryCacheKey key = makeCacheKey(positions[i]);
        auto& bucket = dedup_miss_indices[key];
        if (bucket.empty()) {
            dedup_miss_positions.push_back(positions[i]);
        }
        bucket.push_back(i);
    }

    if (miss_indices.empty()) {
        return results;
    }

    auto request =
        std::make_shared<controller_interfaces::srv::QueryDistanceField::Request>();
    request->positions.reserve(dedup_miss_positions.size());
    for (const auto& p : dedup_miss_positions) {
        geometry_msgs::msg::Point point;
        point.x = p.x();
        point.y = p.y();
        point.z = p.z();
        request->positions.push_back(point);
    }

    std::lock_guard<std::mutex> lock(request_mutex_);
    auto future = client_->async_send_request(request);
    const auto status =
        future.wait_for(std::chrono::milliseconds(config_.request_timeout_ms));
    if (status != std::future_status::ready) {
        if (!logged_timeout_.exchange(true)) {
            RCLCPP_WARN(
                node_->get_logger(),
                "[ReactiveTask] camera_driver ESDF query timed out after %d ms: %s",
                config_.request_timeout_ms,
                config_.service_name.c_str());
        }
        failed_queries_.fetch_add(miss_indices.size(), std::memory_order_relaxed);
        return results;
    }

    const auto response = future.get();
    const std::size_t miss_count = dedup_miss_positions.size();
    if (!response || !response->success || !response->map_ready ||
        response->observed.size() != miss_count ||
        response->distance_valid.size() != miss_count ||
        response->gradient_valid.size() != miss_count ||
        response->distances.size() != miss_count ||
        response->gradients.size() != miss_count) {
        failed_queries_.fetch_add(miss_indices.size(), std::memory_order_relaxed);
        return results;
    }

    for (std::size_t miss_i = 0; miss_i < miss_count; ++miss_i) {
        DistanceFieldQueryResult query;
        query.observed = response->observed[miss_i];
        query.distance_valid = response->distance_valid[miss_i];
        query.gradient_valid = response->gradient_valid[miss_i];
        query.distance = response->distances[miss_i];
        query.gradient = Eigen::Vector3d(
            response->gradients[miss_i].x,
            response->gradients[miss_i].y,
            response->gradients[miss_i].z);

        const QueryCacheKey key = makeCacheKey(dedup_miss_positions[miss_i]);
        const auto bucket_it = dedup_miss_indices.find(key);
        if (bucket_it == dedup_miss_indices.end()) {
            continue;
        }
        for (const std::size_t result_index : bucket_it->second) {
            results[result_index] = query;
        }
        storeCachedResult(dedup_miss_positions[miss_i], query);
    }

    successful_queries_.fetch_add(miss_indices.size(), std::memory_order_relaxed);
    return results;
}

bool CameraDriverEsdfMapClient::isServiceReady() const {
    return client_ && client_->service_is_ready();
}

CameraDriverEsdfMapClient::QueryCacheKey
CameraDriverEsdfMapClient::makeCacheKey(const Eigen::Vector3d& p) const {
    QueryCacheKey key;
    static_assert(sizeof(double) == sizeof(std::uint64_t));
    const double x = p.x();
    const double y = p.y();
    const double z = p.z();
    std::memcpy(&key.x_bits, &x, sizeof(double));
    std::memcpy(&key.y_bits, &y, sizeof(double));
    std::memcpy(&key.z_bits, &z, sizeof(double));
    return key;
}

bool CameraDriverEsdfMapClient::tryGetCachedResult(
    const Eigen::Vector3d& p,
    DistanceFieldQueryResult* result) const {
    if (result == nullptr) {
        return false;
    }
    const QueryCacheKey key = makeCacheKey(p);
    std::lock_guard<std::mutex> lock(cache_mutex_);
    const auto it = query_cache_.find(key);
    if (it == query_cache_.end()) {
        return false;
    }
    *result = it->second;
    return true;
}

void CameraDriverEsdfMapClient::storeCachedResult(
    const Eigen::Vector3d& p,
    const DistanceFieldQueryResult& result) const {
    std::lock_guard<std::mutex> lock(cache_mutex_);
    if (query_cache_.size() >= config_.cache_max_entries) {
        query_cache_.clear();
    }
    query_cache_[makeCacheKey(p)] = result;
}

}  // namespace arm_controller::algorithm::cartesian_path_planner
