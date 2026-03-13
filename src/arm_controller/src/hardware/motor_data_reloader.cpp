#include "arm_controller/hardware/motor_data_reloader.hpp"
#include <fstream>
#include <sstream>

MotorDataReloader::MotorDataReloader(rclcpp::Node::SharedPtr node)
    : node_(node) {}

bool MotorDataReloader::load_trajectory_from_csv(
    const std::string& file_path,
    std::map<std::string, std::vector<double>>& times,
    std::map<std::string, std::vector<std::vector<double>>>& positions,
    std::map<std::string, std::vector<std::vector<double>>>& velocities) {

    std::ifstream file(file_path);
    if (!file.is_open()) {
        RCLCPP_ERROR(node_->get_logger(), "❎ Failed to open file: %s", file_path.c_str());
        return false;
    }

    std::string line;
    std::getline(file, line);  // 跳过表头

    std::map<std::string, double> t0;

    int line_count = 0;
    while (std::getline(file, line)) {
        line_count++;

        std::string interface;
        double timestamp;

        std::vector<double> pos, vel;
        if (!parse_csv_line(line, line_count, interface, timestamp, pos, vel)) {
            RCLCPP_ERROR(node_->get_logger(), "❎ Failed to parse line %d: %s", line_count, line.c_str());
            continue;  // 跳过解析失败的行
        }
        if (!t0.count(interface)) {
            t0[interface] = timestamp;
        }
        double t = timestamp - t0[interface];

        times[interface].push_back(t);
        positions[interface].push_back(pos);
        velocities[interface].push_back(vel);
    }

    file.close();

    for(auto& kv : times)
    {
        RCLCPP_INFO(node_->get_logger(),
            "Loaded %zu points for %s",
            kv.second.size(),
            kv.first.c_str());
    }
    return true;
}

bool MotorDataReloader::parse_csv_line(const std::string& line,
                                     int line_count,
                                     std::string& interface, 
                                     double& timestamp,
                                     std::vector<double>& positions,
                                     std::vector<double>& velocities) {
    try {
        std::stringstream ss(line);
        std::string token;

        // 读取 timestamp (第1列)
        std::getline(ss, token, ',');
        timestamp = std::stod(token);

        // 读取 interface (第2列)
        std::getline(ss, interface, ',');

        // 读取 6个 position 数据 (第3-8列)
        positions.clear();
        for (int i = 0; i < 6; i++) {
            std::getline(ss, token, ',');
            positions.push_back(std::stod(token));
        }

        // 读取 6个 velocity 数据 (第9-14列)
        velocities.clear();
        for (int i = 0; i < 6; i++) {
            std::getline(ss, token, ',');
            velocities.push_back(std::stod(token));
        }
        return true;

    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "❎ Line %d: Exception: %s", line_count, e.what());
        return false;
    }
}
