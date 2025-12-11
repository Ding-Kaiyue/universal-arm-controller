#include "arm_controller/utils/joint_recorder.hpp"

JointRecorder::JointRecorder(double record_rate_hz)
    : recording_(false)
    , last_record_time_(0, 0, RCL_ROS_TIME)
    , first_record_time_(0, 0, RCL_ROS_TIME)
    , stop_flag_(false)
{
    if (record_rate_hz <= 0.0) {
        record_period_ = 0.01;  // 默认 100Hz
    } else {
        record_period_ = 1.0 / record_rate_hz;
    }
}

JointRecorder::~JointRecorder() {
    if (recording_) {
        stop();
    }
}

bool JointRecorder::start(const std::string& file_path,
                          const sensor_msgs::msg::JointState::SharedPtr& sample_msg) {
    if (recording_) {
        RCLCPP_WARN(rclcpp::get_logger("JointRecorder"), "Already recording!");
        return false;
    }

    // 如果目录不存在则创建
    auto dir = std::filesystem::path(file_path).parent_path();
    RCLCPP_INFO(rclcpp::get_logger("JointRecorder"), "Saving to path: %s", dir.c_str());
    if (!std::filesystem::exists(dir)) {
        std::filesystem::create_directories(dir);
    }

    // 如果文件存在则删除
    if (std::filesystem::exists(file_path)) {
        std::filesystem::remove(file_path);
    }

    // 打开文件进行写入
    file_.open(file_path, std::ios::out);
    if (!file_.is_open()) {
        RCLCPP_ERROR(rclcpp::get_logger("JointRecorder"), "Failed to open file: %s", file_path.c_str());
        return false;
    }

    // 写入表头
    file_ << "timestamp";
    for (size_t i = 0; i < sample_msg->position.size(); i++) {
        file_ << ",pos" << i;
    }
    for (size_t i = 0; i < sample_msg->velocity.size(); i++) {
        file_ << ",vel" << i;
    }
    for (size_t i = 0; i < sample_msg->effort.size(); i++) {
        file_ << ",effort" << i;
    }
    file_ << "\n";

    recording_ = true;
    stop_flag_ = false;
    last_record_time_ = rclcpp::Clock(RCL_ROS_TIME).now();
    first_record_time_ = last_record_time_;
    rec_file_path_ = file_path;

    RCLCPP_INFO(rclcpp::get_logger("JointRecorder"), "Started recording to %s", file_path.c_str());

    // 启动后台写入线程
    writer_thread_ = std::thread(&JointRecorder::writerLoop, this);

    return true;
}

void JointRecorder::stop() {
    if (!recording_) {
        return;
    }

    // 通知写入线程停止
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        stop_flag_ = true;
    }
    queue_cv_.notify_all();

    // 等待写入线程完成
    if (writer_thread_.joinable()) {
        writer_thread_.join();
    }

    file_.close();
    recording_ = false;

    RCLCPP_INFO(rclcpp::get_logger("JointRecorder"), "Stopped recording to %s", rec_file_path_.c_str());
}

void JointRecorder::enqueue(const sensor_msgs::msg::JointState::SharedPtr& msg) {
    if (!recording_) {
        return;
    }

    rclcpp::Time msg_time(msg->header.stamp);
    double dt = (msg_time - last_record_time_).seconds();

    // 如果间隔时间不够则跳过
    if (dt < record_period_) {
        return;
    }

    last_record_time_ = msg_time;

    // 将消息加入队列
    std::lock_guard<std::mutex> lock(queue_mutex_);
    msg_queue_.push(msg);
    queue_cv_.notify_one();
}

void JointRecorder::writerLoop() {
    while (true) {
        std::unique_lock<std::mutex> lock(queue_mutex_);
        queue_cv_.wait(lock, [this]() {
            return !msg_queue_.empty() || stop_flag_;
        });

        if (stop_flag_ && msg_queue_.empty()) {
            break;
        }

        auto msg = msg_queue_.front();
        msg_queue_.pop();
        lock.unlock();

        rclcpp::Time msg_time(msg->header.stamp);

        // 计算相对于起始时间的时间戳
        double t = (msg_time - first_record_time_).seconds();

        // 写入数据到文件
        file_ << std::fixed << std::setprecision(6) << t;
        for (auto p : msg->position) {
            file_ << "," << p;
        }
        for (auto v : msg->velocity) {
            file_ << "," << v;
        }
        for (auto e : msg->effort) {
            file_ << "," << e;
        }
        file_ << std::endl;
    }
}

// 移动平均滤波函数（lambda）
namespace {
auto movingAverage = [](const std::vector<double>& data, int window_size = 10) {
    std::vector<double> result(data.size());
    int half = window_size / 2;
    for (size_t i = 0; i < data.size(); i++) {
        double sum = 0.0;
        int count = 0;
        for (int j = -half; j <= half; j++) {
            int idx = static_cast<int>(i) + j;
            if (idx >= 0 && idx < static_cast<int>(data.size())) {
                sum += data[idx];
                count++;
            }
        }
        result[i] = sum / count;
    }
    return result;
};
}  // namespace

trajectory_msgs::msg::JointTrajectory JointRecorder::repeat(const std::string& txt_file_path) {
    std::ifstream ifs(txt_file_path);
    if (!ifs.is_open()) {
        RCLCPP_ERROR(rclcpp::get_logger("JointRecorder"), "Failed to open trajectory file: %s", txt_file_path.c_str());
        return trajectory_msgs::msg::JointTrajectory();
    }

    std::string line;
    // 读取表头
    std::getline(ifs, line);

    trajectory_msgs::msg::JointTrajectory traj_msg;
    traj_msg.joint_names = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};

    // 第一行时间，用于相对时间戳
    double t0 = 0.0;
    bool first_line = true;
    std::vector<double> times;
    std::vector<std::vector<double>> all_positions(6);
    std::vector<std::vector<double>> all_velocities(6);
    std::vector<std::vector<double>> all_efforts(6);

    // 读取所有点
    while (std::getline(ifs, line)) {
        std::stringstream ss(line);
        std::string token;

        // 读取时间戳
        std::getline(ss, token, ',');
        double t = std::stod(token);
        if (first_line) {
            t0 = t;
            first_line = false;
        }

        // 读取位置
        for (size_t i = 0; i < 6; i++) {
            if (!std::getline(ss, token, ',')) break;
            all_positions[i].push_back(std::stod(token));
        }

        // 读取速度
        for (size_t i = 0; i < 6; i++) {
            if (!std::getline(ss, token, ',')) break;
            all_velocities[i].push_back(std::stod(token) / 57.3);  // 如果需要，转换为 rad/s
        }

        // 读取力矩（可选）
        for (size_t i = 0; i < 6; i++) {
            if (!std::getline(ss, token, ',')) break;
            all_efforts[i].push_back(std::stod(token));
        }

        // 存储时间
        times.push_back(t - t0);
    }

    ifs.close();

    // 可选：对速度应用移动平均滤波
    // 如果需要滤波，取消注释以下代码
    // for (size_t i = 0; i < 6; i++) {
    //     all_velocities[i] = movingAverage(all_velocities[i], 5);
    // }

    // 构造 JointTrajectory
    for (size_t idx = 0; idx < times.size(); idx++) {
        trajectory_msgs::msg::JointTrajectoryPoint point;
        for (size_t j = 0; j < 6; j++) {
            point.positions.push_back(all_positions[j][idx]);
            point.velocities.push_back(all_velocities[j][idx]);
            point.effort.push_back(all_efforts[j].empty() ? 0.0 : all_efforts[j][idx]);
        }
        point.time_from_start = rclcpp::Duration::from_seconds(times[idx]);
        traj_msg.points.push_back(point);
    }

    RCLCPP_INFO(rclcpp::get_logger("JointRecorder"), "Loaded trajectory with %zu points", traj_msg.points.size());
    return traj_msg;
}
