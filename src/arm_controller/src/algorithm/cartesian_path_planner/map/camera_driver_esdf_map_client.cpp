#include "algorithm/cartesian_path_planner/map/camera_driver_esdf_map_client.hpp"

#include <chrono>
#include <future>
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

    auto request =
        std::make_shared<controller_interfaces::srv::QueryDistanceField::Request>();
    request->positions.reserve(positions.size());
    for (const auto& p : positions) {
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
        client_->remove_pending_request(future);
        failed_queries_.fetch_add(positions.size(), std::memory_order_relaxed);
        return results;
    }

    const auto response = future.get();
    const std::size_t count = positions.size();
    if (!response || !response->success || !response->map_ready ||
        response->observed.size() != count ||
        response->distance_valid.size() != count ||
        response->gradient_valid.size() != count ||
        response->distances.size() != count ||
        response->gradients.size() != count) {
        failed_queries_.fetch_add(positions.size(), std::memory_order_relaxed);
        return results;
    }

    for (std::size_t i = 0; i < count; ++i) {
        DistanceFieldQueryResult query;
        query.observed = response->observed[i];
        query.distance_valid = response->distance_valid[i];
        query.gradient_valid = response->gradient_valid[i];
        query.distance = response->distances[i];
        query.gradient = Eigen::Vector3d(
            response->gradients[i].x,
            response->gradients[i].y,
            response->gradients[i].z);
        results[i] = query;
    }

    successful_queries_.fetch_add(positions.size(), std::memory_order_relaxed);
    return results;
}

bool CameraDriverEsdfMapClient::isServiceReady() const {
    return client_ && client_->service_is_ready();
}

}  // namespace arm_controller::algorithm::cartesian_path_planner
