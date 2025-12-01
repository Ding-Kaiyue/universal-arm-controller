#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include "controller_interfaces/srv/sdk_cmd.hpp"
#include "controller_interfaces/srv/work_mode.hpp"
#include "robot_sdk/sdk_commands.hpp"

#include <mutex>
#include <vector>
#include <string>
#include <csignal>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

class RobotSDKNode : public rclcpp::Node
{
public:
    RobotSDKNode() : Node("robot_sdk"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
    {
        RCLCPP_INFO(this->get_logger(), "Robot SDK node starting...");

        // 创建回调组 - 使用可重入回调组允许并行处理
        callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

        // 创建SDK服务（使用默认回调组）
        sdk_service_ = this->create_service<controller_interfaces::srv::SdkCmd>(
            "/sdk_cmd",
            std::bind(&RobotSDKNode::handle_sdk_cmd, this,
                      std::placeholders::_1, std::placeholders::_2));

        // 创建模式切换客户端（使用可重入回调组）
        mode_client_ = this->create_client<controller_interfaces::srv::WorkMode>(
            "/controller_api/controller_mode",
            rmw_qos_profile_services_default,
            callback_group_);

        // 订阅关节状态
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&RobotSDKNode::joint_state_callback, this, std::placeholders::_1));

        // 创建示教/复现话题发布者
        teach_pub_ = this->create_publisher<std_msgs::msg::String>(
            "/controller_api/trajectory_record_action", 10);
        replay_pub_ = this->create_publisher<std_msgs::msg::String>(
            "/controller_api/trajectory_replay_action", 10);

        // 等待模式切换服务
        while (!mode_client_->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for service");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Waiting for controller_mode service...");
        }

        RCLCPP_INFO(this->get_logger(), "Robot SDK node started successfully");
    }

    ~RobotSDKNode()
    {
        // 清理所有子进程
        stop_gravity_compensator();
        stop_smoother_node();
    }

private:
    // ========== 子进程管理 ==========

    void start_gravity_compensator()
    {
        if (gravity_compensator_pid_ > 0) {
            RCLCPP_DEBUG(this->get_logger(), "Gravity compensator already running (pid: %d)", gravity_compensator_pid_);
            return;
        }

        pid_t pid = fork();
        if (pid == 0) {
            // 子进程：使用 bash -c 来启动，确保环境变量正确
            const char* urdf_path = "/home/w/work/robotic_arm_ws/install/robot_description/share/robot_description/urdf/arm380.urdf";
            std::string cmd = "source /home/w/work/robotic_arm_ws/install/setup.bash && "
                              "ros2 launch robot_dynamics gravity_compensator.launch.py urdf_file:=" + std::string(urdf_path);

            execlp("bash", "bash", "-c", cmd.c_str(), nullptr);
            // 如果exec失败
            _exit(1);
        } else if (pid > 0) {
            gravity_compensator_pid_ = pid;
            RCLCPP_INFO(this->get_logger(), "Started gravity_compensator via launch (pid: %d)", pid);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to fork gravity_compensator process");
        }
    }

    void stop_gravity_compensator()
    {
        if (gravity_compensator_pid_ > 0) {
            RCLCPP_INFO(this->get_logger(), "Stopping gravity_compensator (pid: %d)", gravity_compensator_pid_);
            kill(gravity_compensator_pid_, SIGINT);

            // 等待进程结束（最多2秒）
            int status;
            int wait_count = 0;
            while (waitpid(gravity_compensator_pid_, &status, WNOHANG) == 0 && wait_count < 20) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                wait_count++;
            }

            // 如果还没结束，强制终止
            if (wait_count >= 20) {
                kill(gravity_compensator_pid_, SIGKILL);
                waitpid(gravity_compensator_pid_, &status, 0);
            }

            gravity_compensator_pid_ = -1;
            RCLCPP_INFO(this->get_logger(), "Gravity compensator stopped");
        }
    }

    void start_smoother_node()
    {
        if (smoother_node_pid_ > 0) {
            RCLCPP_DEBUG(this->get_logger(), "Smoother node already running (pid: %d)", smoother_node_pid_);
            return;
        }

        pid_t pid = fork();
        if (pid == 0) {
            // 子进程：执行轨迹平滑节点
            execlp("ros2", "ros2", "run", "traj_smoother_py", "smoother_node", nullptr);
            // 如果exec失败
            RCLCPP_ERROR(rclcpp::get_logger("smoother_node"), "Failed to start smoother_node");
            _exit(1);
        } else if (pid > 0) {
            smoother_node_pid_ = pid;
            RCLCPP_INFO(this->get_logger(), "Started smoother_node (pid: %d)", pid);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to fork smoother_node process");
        }
    }

    void stop_smoother_node()
    {
        if (smoother_node_pid_ > 0) {
            RCLCPP_INFO(this->get_logger(), "Stopping smoother_node (pid: %d)", smoother_node_pid_);
            kill(smoother_node_pid_, SIGINT);

            // 等待进程结束（最多2秒）
            int status;
            int wait_count = 0;
            while (waitpid(smoother_node_pid_, &status, WNOHANG) == 0 && wait_count < 20) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                wait_count++;
            }

            // 如果还没结束，强制终止
            if (wait_count >= 20) {
                kill(smoother_node_pid_, SIGKILL);
                waitpid(smoother_node_pid_, &status, 0);
            }

            smoother_node_pid_ = -1;
            RCLCPP_INFO(this->get_logger(), "Smoother node stopped");
        }
    }

    // ========== SDK命令处理 ==========

    void handle_sdk_cmd(
        const std::shared_ptr<controller_interfaces::srv::SdkCmd::Request> request,
        std::shared_ptr<controller_interfaces::srv::SdkCmd::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "SDK command received: %d", request->command);

        std::string mapping = request->mapping.empty() ? "single_arm" : request->mapping;
        response->success = false;

        switch (request->command) {
            case robot_sdk::SDK_TEACH_START:
                response->success = handle_teach_start(mapping, request->trajectory_name);
                break;

            case robot_sdk::SDK_TEACH_STOP:
                response->success = handle_teach_stop(mapping);
                break;

            case robot_sdk::SDK_TEACH_REPEAT:
                response->success = handle_teach_repeat(mapping, request->trajectory_name);
                break;

            case robot_sdk::SDK_GET_POSE:
                response->success = handle_get_pose(response);
                break;

            case robot_sdk::SDK_DISABLE:
            case robot_sdk::SDK_HOLD_STATE:
            case robot_sdk::SDK_BACK_TO_START:
            case robot_sdk::SDK_BACK_TO_INITIAL:
            case robot_sdk::SDK_MOVEJ:
            case robot_sdk::SDK_MOVEL:
            case robot_sdk::SDK_JOINT_CONTROL:
            case robot_sdk::SDK_CARTESIAN:
                response->success = handle_mode_switch(request->command, mapping);
                break;

            default:
                response->message = "Unknown command: " + std::to_string(request->command);
                RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
                break;
        }

        if (response->success) {
            response->message = "Command executed successfully";
        }
    }

    bool handle_teach_start(const std::string& mapping, const std::string& trajectory_name)
    {
        RCLCPP_INFO(this->get_logger(), "handle_teach_start called with mapping: %s, trajectory: %s",
                    mapping.c_str(), trajectory_name.c_str());

        // 1. 停止复现相关节点
        stop_smoother_node();
        RCLCPP_INFO(this->get_logger(), "Step 1: smoother_node stopped");

        // 2. 启动重力补偿节点
        start_gravity_compensator();
        RCLCPP_INFO(this->get_logger(), "Step 2: gravity_compensator started, waiting 500ms...");
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        // 3. 切换到示教模式
        RCLCPP_INFO(this->get_logger(), "Step 3: switching to TrajectoryRecord mode...");
        if (!switch_mode("TrajectoryRecord", mapping)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to switch to TrajectoryRecord mode");
            stop_gravity_compensator();
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "Step 3: mode switched successfully");

        // 4. 等待模式切换完成
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        RCLCPP_INFO(this->get_logger(), "Step 4: waited 500ms");

        // 5. 发送录制命令（轨迹名称）
        std_msgs::msg::String msg;
        msg.data = trajectory_name.empty() ? "teach_trajectory" : trajectory_name;
        teach_pub_->publish(msg);

        current_mode_ = "TrajectoryRecord";
        RCLCPP_INFO(this->get_logger(), "Step 5: Started recording trajectory: %s", msg.data.c_str());
        return true;
    }

    bool handle_teach_stop(const std::string& mapping)
    {
        // 1. 发送停止录制命令
        std_msgs::msg::String msg;
        msg.data = "stop";
        teach_pub_->publish(msg);

        // 2. 等待录制停止
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        // 3. 停止重力补偿节点
        stop_gravity_compensator();

        // 4. 切换到保持状态
        switch_mode("HoldState", mapping);

        current_mode_ = "HoldState";
        RCLCPP_INFO(this->get_logger(), "Stopped recording trajectory");
        return true;
    }

    bool handle_teach_repeat(const std::string& mapping, const std::string& trajectory_name)
    {
        // 1. 停止示教相关节点
        stop_gravity_compensator();

        // 2. 启动轨迹平滑节点
        start_smoother_node();
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        // 3. 切换到复现模式
        if (!switch_mode("TrajectoryReplay", mapping)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to switch to TrajectoryReplay mode");
            stop_smoother_node();
            return false;
        }

        // 4. 等待模式切换完成
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        // 5. 发送复现命令（轨迹名称）
        std_msgs::msg::String msg;
        msg.data = trajectory_name.empty() ? "teach_trajectory" : trajectory_name;
        replay_pub_->publish(msg);

        current_mode_ = "TrajectoryReplay";
        RCLCPP_INFO(this->get_logger(), "Started replaying trajectory: %s", msg.data.c_str());
        return true;
    }

    bool handle_get_pose(std::shared_ptr<controller_interfaces::srv::SdkCmd::Response> response)
    {
        std::lock_guard<std::mutex> lock(joint_state_mutex_);

        // 返回当前关节位置
        response->current_joint_positions = current_joint_positions_;

        // 获取末端位姿
        try {
            geometry_msgs::msg::TransformStamped transform =
                tf_buffer_.lookupTransform("world", "Link6", tf2::TimePointZero);

            response->current_cartesian_pose.resize(6);
            response->current_cartesian_pose[0] = transform.transform.translation.x;
            response->current_cartesian_pose[1] = transform.transform.translation.y;
            response->current_cartesian_pose[2] = transform.transform.translation.z;

            tf2::Quaternion quat(
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w);

            double roll, pitch, yaw;
            tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

            response->current_cartesian_pose[3] = roll;
            response->current_cartesian_pose[4] = pitch;
            response->current_cartesian_pose[5] = yaw;

        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN(this->get_logger(), "Transform lookup failed: %s", ex.what());
        }

        RCLCPP_INFO(this->get_logger(), "Returning current pose");
        return true;
    }

    bool handle_mode_switch(uint8_t command, const std::string& mapping)
    {
        const char* mode_name = robot_sdk::get_mode_name(command);
        if (mode_name == nullptr) {
            RCLCPP_ERROR(this->get_logger(), "No mode mapping for command: %d", command);
            return false;
        }

        // 切换到其他模式时，停止示教/复现相关节点
        if (current_mode_ == "TrajectoryRecord") {
            stop_gravity_compensator();
        } else if (current_mode_ == "TrajectoryReplay") {
            stop_smoother_node();
        }

        bool success = switch_mode(mode_name, mapping);
        if (success) {
            current_mode_ = mode_name;
        }
        return success;
    }

    bool switch_mode(const std::string& mode, const std::string& mapping)
    {
        auto request = std::make_shared<controller_interfaces::srv::WorkMode::Request>();
        request->mode = mode;
        request->mapping = mapping;

        RCLCPP_INFO(this->get_logger(), "Sending mode switch request: %s", mode.c_str());

        // 使用 promise/future 配合回调
        auto promise = std::make_shared<std::promise<bool>>();
        auto future = promise->get_future();

        auto callback = [this, promise, mode](
            rclcpp::Client<controller_interfaces::srv::WorkMode>::SharedFuture response_future) {
            try {
                auto response = response_future.get();
                if (response->success) {
                    RCLCPP_INFO(this->get_logger(), "Switched to mode: %s", mode.c_str());
                    promise->set_value(true);
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Failed to switch mode: %s", response->message.c_str());
                    promise->set_value(false);
                }
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Service call exception: %s", e.what());
                promise->set_value(false);
            }
        };

        mode_client_->async_send_request(request, callback);

        // 等待结果（带超时）
        if (future.wait_for(std::chrono::seconds(10)) == std::future_status::ready) {
            return future.get();
        } else {
            RCLCPP_ERROR(this->get_logger(), "Mode switch service timeout");
            return false;
        }
    }

    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(joint_state_mutex_);

        // 按关节名称排序存储
        std::vector<std::string> joint_names = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
        current_joint_positions_.resize(joint_names.size(), 0.0);

        for (size_t i = 0; i < msg->name.size(); ++i) {
            for (size_t j = 0; j < joint_names.size(); ++j) {
                if (msg->name[i] == joint_names[j]) {
                    current_joint_positions_[j] = msg->position[i];
                    break;
                }
            }
        }
    }

    // 服务和客户端
    rclcpp::Service<controller_interfaces::srv::SdkCmd>::SharedPtr sdk_service_;
    rclcpp::Client<controller_interfaces::srv::WorkMode>::SharedPtr mode_client_;

    // 订阅者
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

    // 发布者
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr teach_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr replay_pub_;

    // TF
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // 数据
    std::mutex joint_state_mutex_;
    std::vector<double> current_joint_positions_;

    // 子进程管理
    pid_t gravity_compensator_pid_ = -1;
    pid_t smoother_node_pid_ = -1;
    std::string current_mode_;

    // 回调组
    rclcpp::CallbackGroup::SharedPtr callback_group_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotSDKNode>();

    // 使用多线程执行器，避免服务回调中调用其他服务时死锁
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
