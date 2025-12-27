#include "arm_controller/hardware/motor_data_reloader.hpp"
#include <fstream>
#include <sstream>

MotorDataReloader::MotorDataReloader(rclcpp::Node::SharedPtr node)
    : node_(node) {}

bool MotorDataReloader::load_trajectory_from_csv(
    const std::string& file_path,
    std::vector<double>& times,
    std::vector<std::vector<double>>& positions,
    std::vector<std::vector<double>>& velocities,
    std::vector<std::vector<double>>& efforts) {

    std::ifstream file(file_path);
    if (!file.is_open()) {
        RCLCPP_ERROR(node_->get_logger(), "❎ Failed to open file: %s", file_path.c_str());
        return false;
    }

    std::string line;
    std::getline(file, line);  // 跳过表头

    int line_count = 0;
    while (std::getline(file, line)) {
        line_count++;

        // 去除行尾的空白字符
        while (!line.empty() && (line.back() == '\r' || line.back() == '\n' ||
               line.back() == ' ' || line.back() == '\t')) {
            line.pop_back();
        }

        if (line.empty()) {
            RCLCPP_DEBUG(node_->get_logger(), "⚠️  Line %d: Empty line, skipping", line_count);
            continue;
        }

        double timestamp;
        std::vector<double> pos, vel, eff;

        if (!parse_csv_line(line, line_count, timestamp, pos, vel, eff)) {
            continue;
        }

        times.push_back(timestamp);
        positions.push_back(pos);
        velocities.push_back(vel);
        efforts.push_back(eff);
    }

    file.close();

    if (positions.empty()) {
        RCLCPP_ERROR(node_->get_logger(), "❎ Empty trajectory");
        return false;
    }

    loaded_file_ = file_path;
    total_points_ = positions.size();

    RCLCPP_INFO(node_->get_logger(), "✅ Loaded %zu poses from %d lines", total_points_, line_count);
    return true;
}

bool MotorDataReloader::parse_csv_line(const std::string& line, int line_count,
                                     double& timestamp,
                                     std::vector<double>& positions,
                                     std::vector<double>& velocities,
                                     std::vector<double>& efforts) {
    try {
        std::stringstream ss(line);
        std::string token;

        // 读取 timestamp (第1列)
        if (!std::getline(ss, token, ',')) {
            RCLCPP_WARN(node_->get_logger(), "❎ Line %d: Failed to read timestamp", line_count);
            return false;
        }
        token.erase(0, token.find_first_not_of(" \t"));
        token.erase(token.find_last_not_of(" \t") + 1);
        timestamp = std::stod(token);

        // 读取 interface (第2列) - 跳过
        std::getline(ss, token, ',');

        // 读取 6个 position 数据 (第3-8列)
        positions.reserve(6);
        for (int i = 0; i < 6; i++) {
            if (!std::getline(ss, token, ',')) {
                RCLCPP_WARN(node_->get_logger(), "❎ Line %d: Failed to read position%d", line_count, i);
                return false;
            }
            token.erase(0, token.find_first_not_of(" \t"));
            token.erase(token.find_last_not_of(" \t") + 1);
            positions.push_back(std::stod(token));
        }

        // 读取 6个 velocity 数据 (第9-14列)
        velocities.reserve(6);
        for (int i = 0; i < 6; i++) {
            if (!std::getline(ss, token, ',')) {
                RCLCPP_WARN(node_->get_logger(), "❎ Line %d: Failed to read velocity%d", line_count, i);
                return false;
            }
            token.erase(0, token.find_first_not_of(" \t"));
            token.erase(token.find_last_not_of(" \t") + 1);
            velocities.push_back(std::stod(token));
        }

        // 读取 6个 effort 数据 (第15-20列)
        efforts.reserve(6);
        for (int i = 0; i < 6; i++) {
            if (!std::getline(ss, token, ',')) {
                RCLCPP_WARN(node_->get_logger(), "❎ Line %d: Failed to read effort%d", line_count, i);
                return false;
            }
            token.erase(0, token.find_first_not_of(" \t"));
            token.erase(token.find_last_not_of(" \t") + 1);
            efforts.push_back(std::stod(token));
        }

        return true;

    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "❎ Line %d: Exception: %s", line_count, e.what());
        return false;
    }
}
