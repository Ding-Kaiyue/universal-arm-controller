#ifndef __MOTOR_DATA_RELOADER_HPP__
#define __MOTOR_DATA_RELOADER_HPP__

#include <vector>
#include <string>
#include <memory>
#include <rclcpp/rclcpp.hpp>

/**
 * @brief 电机数据重放加载器
 * 负责从CSV文件加载电机轨迹数据
 */
class MotorDataReloader {
public:
    explicit MotorDataReloader(rclcpp::Node::SharedPtr node);
    ~MotorDataReloader() = default;

    /**
     * @brief 从CSV文件加载轨迹数据
     * @param file_path CSV文件路径
     * @param times 时间戳数组（输出）
     * @param positions 位置数据数组（输出）
     * @param velocities 速度数据数组（输出）
     * @param efforts 力矩数据数组（输出）
     * @return 加载成功返回true，失败返回false
     */
    bool load_trajectory_from_csv(
        const std::string& file_path,
        std::vector<double>& times,
        std::vector<std::vector<double>>& positions,
        std::vector<std::vector<double>>& velocities,
        std::vector<std::vector<double>>& efforts);

    /**
     * @brief 获取加载的数据点总数
     */
    size_t get_total_points() const { return total_points_; }

    /**
     * @brief 获取加载的文件路径
     */
    const std::string& get_loaded_file() const { return loaded_file_; }

private:
    rclcpp::Node::SharedPtr node_;
    size_t total_points_ = 0;
    std::string loaded_file_;

    // 辅助函数：解析CSV行
    bool parse_csv_line(const std::string& line, int line_count,
                       double& timestamp,
                       std::vector<double>& positions,
                       std::vector<double>& velocities,
                       std::vector<double>& efforts);
};

#endif // __MOTOR_DATA_RELOADER_HPP__
