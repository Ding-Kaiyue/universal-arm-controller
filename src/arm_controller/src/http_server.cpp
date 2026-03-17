#include "arm_controller/arm_controller_api.hpp"
#include "controller/movej/movej_ipc_interface.hpp"
#include "controller/movel/movel_ipc_interface.hpp"
#include "controller/movec/movec_ipc_interface.hpp"
#include "controller/joint_velocity/joint_velocity_ipc_interface.hpp"
#include "controller/cartesian_velocity/cartesian_velocity_ipc_interface.hpp"
#include "controller/trajectory_record/trajectory_record_ipc_interface.hpp"
#include "controller/trajectory_replay/trajectory_replay_ipc_interface.hpp"
#include "controller/basic_ops/basic_ops_ipc_interface.hpp"
#include <iostream>
#include <thread>
#include <memory>
#include <cstring>
#include <sstream>
#include <vector>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <signal.h>
#include <cctype>

// 简单的 JSON 构造器（不使用外部库）
class SimpleJSON {
public:
    static std::string success(const std::string& message, const std::string& mode = "", int state = 0) {
        std::ostringstream oss;
        oss << "{\"status\":\"success\",\"message\":\"" << message << "\"";
        if (!mode.empty()) {
            oss << ",\"mode\":\"" << mode << "\"";
        }
        if (state >= 0) {
            oss << ",\"execution_state\":" << state;
        }
        oss << "}";
        return oss.str();
    }

    static std::string error(const std::string& error_msg) {
        std::ostringstream oss;
        oss << "{\"status\":\"error\",\"error\":\"" << error_msg << "\"}";
        return oss.str();
    }

    static std::string ok() {
        return "{\"status\":\"ok\"}";
    }
};

// 简单的请求体解析器
class RequestParser {
public:
    struct MoveJRequest {
        std::vector<double> positions;
        std::string mapping;
        bool wait_for_result = false;
        int timeout_ms = 15000;
    };

    struct MoveLRequest {
        double x, y, z;
        double qx, qy, qz, qw;
        std::string mapping;
        bool wait_for_result = false;
        int timeout_ms = 15000;
    };

    struct MoveCRequest {
        std::vector<double> waypoints;
        std::string mapping;
        bool wait_for_result = false;
        int timeout_ms = 15000;
    };

    struct VelocityRequest {
        std::vector<double> values;
        std::string mapping;
        int duration_ms = 0;   // 0 = single-shot
        int interval_ms = 5;   // used when duration_ms > 0
        bool auto_stop = true; // send zero velocity when stream ends
    };

    struct GripperRequest {
        std::string mapping;
        int position = 0;        // raw [0,255], 0=夹紧,255=张开
        int velocity = 128;      // raw [0,255]
        int effort = 128;        // raw [0,255]
        int gripper_type = -1;   // 0=OmniPicker,1=PGC,-1=auto by mapping
    };

    struct MotorSwitchRequest {
        std::string mapping;
        int mode = 1;
    };

    struct TeachRequest {
        std::string action;
        std::string filename;
        std::string mapping;
    };

    static MoveJRequest parseMoveJ(const std::string& body) {
        MoveJRequest req;
        req.mapping = "single_arm";  // 默认值

        // 查找 "positions" 字段
        size_t pos_start = body.find("\"positions\"");
        if (pos_start == std::string::npos) {
            throw std::runtime_error("Missing 'positions' field");
        }

        // 查找 [
        size_t array_start = body.find('[', pos_start);
        size_t array_end = body.find(']', array_start);
        if (array_start == std::string::npos || array_end == std::string::npos) {
            throw std::runtime_error("Invalid 'positions' array format");
        }

        std::string array_str = body.substr(array_start + 1, array_end - array_start - 1);
        // 解析数字
        std::istringstream iss(array_str);
        std::string token;
        while (std::getline(iss, token, ',')) {
            // 移除空白和引号
            token.erase(0, token.find_first_not_of(" \t\n\r"));
            token.erase(token.find_last_not_of(" \t\n\r") + 1);
            if (!token.empty() && token != "[" && token != "]") {
                try {
                    req.positions.push_back(std::stod(token));
                } catch (...) {
                    // 忽略解析失败的值
                }
            }
        }

        // 查找 "mapping" 字段
        size_t map_start = body.find("\"mapping\"");
        if (map_start != std::string::npos) {
            size_t colon_pos = body.find(':', map_start);
            size_t quote_start = body.find('"', colon_pos);
            size_t quote_end = body.find('"', quote_start + 1);
            if (quote_start != std::string::npos && quote_end != std::string::npos) {
                req.mapping = body.substr(quote_start + 1, quote_end - quote_start - 1);
            }
        }

        if (req.positions.empty()) {
            throw std::runtime_error("No valid positions found");
        }

        req.wait_for_result = parseBoolFieldOptional(body, "wait_for_result", false);
        req.timeout_ms = parseIntFieldOptional(body, "timeout_ms", 15000);
        if (req.timeout_ms <= 0) {
            req.timeout_ms = 15000;
        }

        return req;
    }

    static MoveLRequest parseMoveL(const std::string& body) {
        MoveLRequest req;
        req.mapping = "single_arm";
        req.qx = req.qy = req.qz = 0.0;
        req.qw = 1.0;

        // 解析 x, y, z
        req.x = parseDoubleField(body, "x");
        req.y = parseDoubleField(body, "y");
        req.z = parseDoubleField(body, "z");

        // 解析四元数（可选）
        if (body.find("\"qx\"") != std::string::npos) {
            req.qx = parseDoubleField(body, "qx");
            req.qy = parseDoubleField(body, "qy");
            req.qz = parseDoubleField(body, "qz");
            req.qw = parseDoubleField(body, "qw");
        }

        // 查找 mapping
        size_t map_start = body.find("\"mapping\"");
        if (map_start != std::string::npos) {
            size_t colon_pos = body.find(':', map_start);
            size_t quote_start = body.find('"', colon_pos);
            size_t quote_end = body.find('"', quote_start + 1);
            if (quote_start != std::string::npos && quote_end != std::string::npos) {
                req.mapping = body.substr(quote_start + 1, quote_end - quote_start - 1);
            }
        }

        req.wait_for_result = parseBoolFieldOptional(body, "wait_for_result", false);
        req.timeout_ms = parseIntFieldOptional(body, "timeout_ms", 15000);
        if (req.timeout_ms <= 0) {
            req.timeout_ms = 15000;
        }

        return req;
    }

    static MoveCRequest parseMoveC(const std::string& body) {
        MoveCRequest req;
        req.mapping = "single_arm";

        // 查找 "waypoints" 字段
        size_t wp_start = body.find("\"waypoints\"");
        if (wp_start == std::string::npos) {
            throw std::runtime_error("Missing 'waypoints' field");
        }

        size_t array_start = body.find('[', wp_start);
        size_t array_end = body.find(']', array_start);
        if (array_start == std::string::npos || array_end == std::string::npos) {
            throw std::runtime_error("Invalid 'waypoints' array format");
        }

        std::string array_str = body.substr(array_start + 1, array_end - array_start - 1);
        std::istringstream iss(array_str);
        std::string token;
        while (std::getline(iss, token, ',')) {
            token.erase(0, token.find_first_not_of(" \t\n\r"));
            token.erase(token.find_last_not_of(" \t\n\r") + 1);
            if (!token.empty() && token != "[" && token != "]") {
                try {
                    req.waypoints.push_back(std::stod(token));
                } catch (...) {
                    // 忽略解析失败的值
                }
            }
        }

        // 查找 mapping
        size_t map_start = body.find("\"mapping\"");
        if (map_start != std::string::npos) {
            size_t colon_pos = body.find(':', map_start);
            size_t quote_start = body.find('"', colon_pos);
            size_t quote_end = body.find('"', quote_start + 1);
            if (quote_start != std::string::npos && quote_end != std::string::npos) {
                req.mapping = body.substr(quote_start + 1, quote_end - quote_start - 1);
            }
        }

        if (req.waypoints.empty()) {
            throw std::runtime_error("No valid waypoints found");
        }

        req.wait_for_result = parseBoolFieldOptional(body, "wait_for_result", false);
        req.timeout_ms = parseIntFieldOptional(body, "timeout_ms", 15000);
        if (req.timeout_ms <= 0) {
            req.timeout_ms = 15000;
        }

        return req;
    }

    static VelocityRequest parseJointVelocity(const std::string& body) {
        VelocityRequest req;
        req.mapping = "single_arm";

        // 查找 "joint_velocities" 字段
        size_t jv_start = body.find("\"joint_velocities\"");
        if (jv_start == std::string::npos) {
            throw std::runtime_error("Missing 'joint_velocities' field");
        }

        req.values = parseDoubleArray(body, jv_start);

        // 查找 mapping
        size_t map_start = body.find("\"mapping\"");
        if (map_start != std::string::npos) {
            size_t colon_pos = body.find(':', map_start);
            size_t quote_start = body.find('"', colon_pos);
            size_t quote_end = body.find('"', quote_start + 1);
            if (quote_start != std::string::npos && quote_end != std::string::npos) {
                req.mapping = body.substr(quote_start + 1, quote_end - quote_start - 1);
            }
        }

        if (req.values.empty()) {
            throw std::runtime_error("No valid joint velocities found");
        }

        req.duration_ms = parseIntFieldOptional(body, "duration_ms", 0);
        req.interval_ms = parseIntFieldOptional(body, "interval_ms", 5);
        req.auto_stop = parseBoolFieldOptional(body, "auto_stop", true);
        if (req.interval_ms <= 0) {
            req.interval_ms = 5;
        }
        if (req.duration_ms < 0) {
            req.duration_ms = 0;
        }

        return req;
    }

    static VelocityRequest parseCartesianVelocity(const std::string& body) {
        VelocityRequest req;
        req.mapping = "single_arm";

        // 查找 "cartesian_velocities" 字段
        size_t cv_start = body.find("\"cartesian_velocities\"");
        if (cv_start == std::string::npos) {
            throw std::runtime_error("Missing 'cartesian_velocities' field");
        }

        req.values = parseDoubleArray(body, cv_start);

        // 查找 mapping
        size_t map_start = body.find("\"mapping\"");
        if (map_start != std::string::npos) {
            size_t colon_pos = body.find(':', map_start);
            size_t quote_start = body.find('"', colon_pos);
            size_t quote_end = body.find('"', quote_start + 1);
            if (quote_start != std::string::npos && quote_end != std::string::npos) {
                req.mapping = body.substr(quote_start + 1, quote_end - quote_start - 1);
            }
        }

        if (req.values.empty()) {
            throw std::runtime_error("No valid cartesian velocities found");
        }

        req.duration_ms = parseIntFieldOptional(body, "duration_ms", 0);
        req.interval_ms = parseIntFieldOptional(body, "interval_ms", 5);
        req.auto_stop = parseBoolFieldOptional(body, "auto_stop", true);
        if (req.interval_ms <= 0) {
            req.interval_ms = 5;
        }
        if (req.duration_ms < 0) {
            req.duration_ms = 0;
        }

        return req;
    }

    static TeachRequest parseTeach(const std::string& body) {
        TeachRequest req;
        req.action = parseStringField(body, "action");
        req.mapping = parseStringFieldOptional(body, "mapping", "*");
        req.filename = parseStringFieldOptional(body, "filename", "trajectory_demo");
        return req;
    }

    static GripperRequest parseGripper(const std::string& body) {
        GripperRequest req;
        req.mapping = parseStringFieldOptional(body, "mapping", "left_gripper");
        req.position = parseIntFieldOptional(body, "position", 0);
        req.velocity = parseIntFieldOptional(body, "velocity", 128);
        req.effort = parseIntFieldOptional(body, "effort", 128);
        req.gripper_type = parseIntFieldOptional(body, "gripper_type", -1);
        return req;
    }

    static MotorSwitchRequest parseMotorSwitch(const std::string& body) {
        MotorSwitchRequest req;
        req.mapping = parseStringField(body, "mapping");
        req.mode = parseIntFieldOptional(body, "mode", 1);
        return req;
    }

private:
    static double parseDoubleField(const std::string& body, const std::string& field_name) {
        size_t field_pos = body.find("\"" + field_name + "\"");
        if (field_pos == std::string::npos) {
            throw std::runtime_error("Missing field: " + field_name);
        }

        size_t colon_pos = body.find(':', field_pos);
        size_t value_start = body.find_first_not_of(" \t\r\n", colon_pos + 1);
        size_t value_end = value_start;
        while (value_end < body.length() &&
               (std::isdigit(body[value_end]) || body[value_end] == '.' || body[value_end] == '-')) {
            value_end++;
        }

        std::string value_str = body.substr(value_start, value_end - value_start);
        return std::stod(value_str);
    }

    static std::vector<double> parseDoubleArray(const std::string& body, size_t field_start) {
        std::vector<double> values;
        size_t array_start = body.find('[', field_start);
        size_t array_end = body.find(']', array_start);

        if (array_start == std::string::npos || array_end == std::string::npos) {
            throw std::runtime_error("Invalid array format");
        }

        std::string array_str = body.substr(array_start + 1, array_end - array_start - 1);
        std::istringstream iss(array_str);
        std::string token;
        while (std::getline(iss, token, ',')) {
            token.erase(0, token.find_first_not_of(" \t\n\r"));
            token.erase(token.find_last_not_of(" \t\n\r") + 1);
            if (!token.empty() && token != "[" && token != "]") {
                try {
                    values.push_back(std::stod(token));
                } catch (...) {
                    // 忽略解析失败的值
                }
            }
        }

        return values;
    }

    static std::string parseStringField(const std::string& body, const std::string& field_name) {
        size_t field_pos = body.find("\"" + field_name + "\"");
        if (field_pos == std::string::npos) {
            throw std::runtime_error("Missing field: " + field_name);
        }
        size_t colon_pos = body.find(':', field_pos);
        size_t quote_start = body.find('"', colon_pos + 1);
        size_t quote_end = body.find('"', quote_start + 1);
        if (quote_start == std::string::npos || quote_end == std::string::npos) {
            throw std::runtime_error("Invalid string field: " + field_name);
        }
        return body.substr(quote_start + 1, quote_end - quote_start - 1);
    }

    static std::string parseStringFieldOptional(
        const std::string& body, const std::string& field_name, const std::string& default_value) {
        size_t field_pos = body.find("\"" + field_name + "\"");
        if (field_pos == std::string::npos) {
            return default_value;
        }
        size_t colon_pos = body.find(':', field_pos);
        size_t quote_start = body.find('"', colon_pos + 1);
        size_t quote_end = body.find('"', quote_start + 1);
        if (quote_start == std::string::npos || quote_end == std::string::npos) {
            return default_value;
        }
        return body.substr(quote_start + 1, quote_end - quote_start - 1);
    }

    static int parseIntFieldOptional(
        const std::string& body, const std::string& field_name, int default_value) {
        size_t field_pos = body.find("\"" + field_name + "\"");
        if (field_pos == std::string::npos) {
            return default_value;
        }

        size_t colon_pos = body.find(':', field_pos);
        if (colon_pos == std::string::npos) {
            return default_value;
        }

        size_t value_start = body.find_first_not_of(" \t\r\n", colon_pos + 1);
        if (value_start == std::string::npos) {
            return default_value;
        }

        size_t value_end = value_start;
        while (value_end < body.length() &&
               (std::isdigit(body[value_end]) || body[value_end] == '-')) {
            value_end++;
        }

        try {
            return std::stoi(body.substr(value_start, value_end - value_start));
        } catch (...) {
            return default_value;
        }
    }

    static bool parseBoolFieldOptional(
        const std::string& body, const std::string& field_name, bool default_value) {
        size_t field_pos = body.find("\"" + field_name + "\"");
        if (field_pos == std::string::npos) {
            return default_value;
        }

        size_t colon_pos = body.find(':', field_pos);
        if (colon_pos == std::string::npos) {
            return default_value;
        }

        size_t value_start = body.find_first_not_of(" \t\r\n", colon_pos + 1);
        if (value_start == std::string::npos) {
            return default_value;
        }

        if (body.compare(value_start, 4, "true") == 0) {
            return true;
        }
        if (body.compare(value_start, 5, "false") == 0) {
            return false;
        }
        return default_value;
    }
};

class SimpleHTTPServer {
private:
    int server_socket_ = -1;
    int port_;
    bool running_ = false;
    arm_controller::movej::MoveJIPCInterface movej_;
    arm_controller::movel::MoveLIPCInterface movel_;
    arm_controller::movec::MoveCIPCInterface movec_;
    arm_controller::joint_velocity::JointVelocityIPCInterface joint_velocity_;
    arm_controller::cartesian_velocity::CartesianVelocityIPCInterface cartesian_velocity_;
    arm_controller::trajectory_record::TrajectoryRecordIPCInterface trajectory_record_;
    arm_controller::trajectory_replay::TrajectoryReplayIPCInterface trajectory_replay_;
    arm_controller::basic_ops::BasicOpsIPCInterface basic_ops_;

public:
    SimpleHTTPServer(int port) : port_(port) {}

    ~SimpleHTTPServer() {
        stop();
    }

    bool start() {
        // 创建 socket
        server_socket_ = socket(AF_INET, SOCK_STREAM, 0);
        if (server_socket_ < 0) {
            std::cerr << "Failed to create socket" << std::endl;
            return false;
        }

        // 设置 SO_REUSEADDR
        int opt = 1;
        if (setsockopt(server_socket_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0) {
            std::cerr << "Failed to set socket option" << std::endl;
            close(server_socket_);
            return false;
        }

        // 绑定
        struct sockaddr_in server_addr{};
        server_addr.sin_family = AF_INET;
        server_addr.sin_addr.s_addr = inet_addr("127.0.0.1");
        server_addr.sin_port = htons(port_);

        if (bind(server_socket_, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
            std::cerr << "Failed to bind socket to port " << port_ << std::endl;
            close(server_socket_);
            return false;
        }

        // 监听
        if (listen(server_socket_, 5) < 0) {
            std::cerr << "Failed to listen on socket" << std::endl;
            close(server_socket_);
            return false;
        }

        running_ = true;
        std::cout << "HTTP Server listening on 127.0.0.1:" << port_ << std::endl;

        // 启动接收线程
        std::thread(&SimpleHTTPServer::acceptLoop, this).detach();
        return true;
    }

    void stop() {
        running_ = false;
        if (server_socket_ >= 0) {
            close(server_socket_);
            server_socket_ = -1;
        }
    }

private:
    static size_t parseContentLength(const std::string& headers) {
        size_t pos = headers.find("Content-Length:");
        if (pos == std::string::npos) {
            pos = headers.find("content-length:");
        }
        if (pos == std::string::npos) {
            return 0;
        }

        size_t colon = headers.find(':', pos);
        if (colon == std::string::npos) {
            return 0;
        }
        size_t start = headers.find_first_not_of(" \t", colon + 1);
        if (start == std::string::npos) {
            return 0;
        }
        size_t end = start;
        while (end < headers.size() && std::isdigit(static_cast<unsigned char>(headers[end]))) {
            ++end;
        }
        try {
            return static_cast<size_t>(std::stoul(headers.substr(start, end - start)));
        } catch (...) {
            return 0;
        }
    }

    template <typename GetStateFn>
    bool wait_until_terminal(const std::string& mapping, int timeout_ms, GetStateFn get_state,
                             arm_controller::ipc::ExecutionState& final_state) {
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
        while (std::chrono::steady_clock::now() < deadline) {
            final_state = get_state(mapping);
            if (final_state == arm_controller::ipc::ExecutionState::SUCCESS ||
                final_state == arm_controller::ipc::ExecutionState::FAILED) {
                return true;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        final_state = get_state(mapping);
        return (final_state == arm_controller::ipc::ExecutionState::SUCCESS ||
                final_state == arm_controller::ipc::ExecutionState::FAILED);
    }

    void acceptLoop() {
        while (running_) {
            struct sockaddr_in client_addr{};
            socklen_t addr_len = sizeof(client_addr);

            int client_socket = accept(server_socket_, (struct sockaddr*)&client_addr, &addr_len);
            if (client_socket < 0) {
                if (running_) {
                    std::cerr << "Accept failed" << std::endl;
                }
                continue;
            }

            // 处理请求
            std::thread(&SimpleHTTPServer::handleClient, this, client_socket).detach();
        }
    }

    void handleClient(int client_socket) {
        std::string request;
        request.reserve(8192);

        char buffer[4096];
        ssize_t received = 0;
        size_t header_end = std::string::npos;
        size_t content_length = 0;
        size_t total_needed = 0;

        while (true) {
            received = recv(client_socket, buffer, sizeof(buffer), 0);
            if (received <= 0) {
                break;
            }
            request.append(buffer, static_cast<size_t>(received));

            if (header_end == std::string::npos) {
                header_end = request.find("\r\n\r\n");
                if (header_end != std::string::npos) {
                    content_length = parseContentLength(request.substr(0, header_end + 4));
                    total_needed = (header_end + 4) + content_length;
                }
            }

            if (header_end != std::string::npos && request.size() >= total_needed) {
                break;
            }
        }

        if (request.empty()) {
            close(client_socket);
            return;
        }

        // 解析请求
        std::string response = handleRequest(request);

        // 发送响应
        send(client_socket, response.c_str(), response.length(), 0);
        close(client_socket);
    }

    std::string handleRequest(const std::string& request) {
        // 简单的 HTTP 请求解析
        std::istringstream iss(request);
        std::string method, path, http_version;
        iss >> method >> path >> http_version;

        // 提取 POST 体
        std::string body;
        size_t body_start = request.find("\r\n\r\n");
        if (body_start != std::string::npos) {
            body = request.substr(body_start + 4);
        }

        std::string response_body;
        std::string http_status;

        try {
            if (method == "POST" && path == "/movej") {
                response_body = handleMoveJ(body);
                http_status = "HTTP/1.1 200 OK";
            } else if (method == "POST" && path == "/movel") {
                response_body = handleMoveL(body);
                http_status = "HTTP/1.1 200 OK";
            } else if (method == "POST" && path == "/movec") {
                response_body = handleMoveC(body);
                http_status = "HTTP/1.1 200 OK";
            } else if (method == "POST" && path == "/joint_velocity") {
                response_body = handleJointVelocity(body);
                http_status = "HTTP/1.1 200 OK";
            } else if (method == "POST" && path == "/cartesian_velocity") {
                response_body = handleCartesianVelocity(body);
                http_status = "HTTP/1.1 200 OK";
            } else if (method == "POST" && path == "/trajectory_record") {
                response_body = handleTrajectoryRecord(body);
                http_status = "HTTP/1.1 200 OK";
            } else if (method == "POST" && path == "/trajectory_replay") {
                response_body = handleTrajectoryReplay(body);
                http_status = "HTTP/1.1 200 OK";
            } else if (method == "POST" && path == "/gripper_control") {
                response_body = handleGripperControl(body);
                http_status = "HTTP/1.1 200 OK";
            } else if (method == "POST" && path == "/motor_enable") {
                response_body = handleMotorEnable(body);
                http_status = "HTTP/1.1 200 OK";
            } else if (method == "POST" && path == "/motor_disable") {
                response_body = handleMotorDisable(body);
                http_status = "HTTP/1.1 200 OK";
            } else if (method == "GET" && path == "/health") {
                response_body = SimpleJSON::ok();
                http_status = "HTTP/1.1 200 OK";
            } else {
                response_body = SimpleJSON::error("Not Found");
                http_status = "HTTP/1.1 404 Not Found";
            }
        } catch (const std::exception& e) {
            response_body = SimpleJSON::error(e.what());
            http_status = "HTTP/1.1 500 Internal Server Error";
        }

        // 构建 HTTP 响应
        std::ostringstream response;
        response << http_status << "\r\n";
        response << "Content-Type: application/json\r\n";
        response << "Content-Length: " << response_body.length() << "\r\n";
        response << "Connection: close\r\n";
        response << "\r\n";
        response << response_body;

        return response.str();
    }

    std::string handleMoveJ(const std::string& body) {
        try {
            auto req = RequestParser::parseMoveJ(body);

            // 执行 MoveJ 命令
            if (movej_.execute(req.positions, req.mapping)) {
                if (req.wait_for_result) {
                    arm_controller::ipc::ExecutionState final_state = arm_controller::ipc::ExecutionState::PENDING;
                    const bool done = wait_until_terminal(
                        req.mapping, req.timeout_ms,
                        [this](const std::string& m) { return movej_.getExecutionState(m); },
                        final_state);
                    if (!done) {
                        return SimpleJSON::success(
                            "Command queued (timeout waiting final result)",
                            movej_.getCurrentMode(req.mapping),
                            static_cast<int>(movej_.getExecutionState(req.mapping)));
                    }
                    if (final_state == arm_controller::ipc::ExecutionState::FAILED) {
                        return SimpleJSON::error("MoveJ execution failed");
                    }
                    return SimpleJSON::success(
                        "Command executed successfully",
                        movej_.getCurrentMode(req.mapping),
                        static_cast<int>(movej_.getExecutionState(req.mapping)));
                }
                return SimpleJSON::success(
                    "Command queued",
                    movej_.getCurrentMode(req.mapping),
                    static_cast<int>(movej_.getExecutionState(req.mapping))
                );
            } else {
                return SimpleJSON::error(movej_.getLastError());
            }
        } catch (const std::exception& e) {
            return SimpleJSON::error(e.what());
        }
    }

    std::string handleMoveL(const std::string& body) {
        try {
            auto req = RequestParser::parseMoveL(body);

            // 执行 MoveL 命令
            if (movel_.execute(req.x, req.y, req.z, req.qx, req.qy, req.qz, req.qw, req.mapping)) {
                if (req.wait_for_result) {
                    arm_controller::ipc::ExecutionState final_state = arm_controller::ipc::ExecutionState::PENDING;
                    const bool done = wait_until_terminal(
                        req.mapping, req.timeout_ms,
                        [this](const std::string& m) { return movel_.getExecutionState(m); },
                        final_state);
                    if (!done) {
                        return SimpleJSON::success(
                            "Command queued (timeout waiting final result)",
                            movel_.getCurrentMode(req.mapping),
                            static_cast<int>(movel_.getExecutionState(req.mapping)));
                    }
                    if (final_state == arm_controller::ipc::ExecutionState::FAILED) {
                        return SimpleJSON::error("MoveL execution failed");
                    }
                    return SimpleJSON::success(
                        "Command executed successfully",
                        movel_.getCurrentMode(req.mapping),
                        static_cast<int>(movel_.getExecutionState(req.mapping)));
                }
                return SimpleJSON::success(
                    "Command queued",
                    movel_.getCurrentMode(req.mapping),
                    static_cast<int>(movel_.getExecutionState(req.mapping))
                );
            } else {
                return SimpleJSON::error("MoveL execution failed");
            }
        } catch (const std::exception& e) {
            return SimpleJSON::error(e.what());
        }
    }

    std::string handleMoveC(const std::string& body) {
        try {
            auto req = RequestParser::parseMoveC(body);

            // 执行 MoveC 命令
            if (movec_.execute(req.waypoints, req.mapping)) {
                if (req.wait_for_result) {
                    arm_controller::ipc::ExecutionState final_state = arm_controller::ipc::ExecutionState::PENDING;
                    const bool done = wait_until_terminal(
                        req.mapping, req.timeout_ms,
                        [this](const std::string& m) { return movec_.getExecutionState(m); },
                        final_state);
                    if (!done) {
                        return SimpleJSON::success(
                            "Command queued (timeout waiting final result)",
                            movec_.getCurrentMode(req.mapping),
                            static_cast<int>(movec_.getExecutionState(req.mapping)));
                    }
                    if (final_state == arm_controller::ipc::ExecutionState::FAILED) {
                        return SimpleJSON::error("MoveC execution failed");
                    }
                    return SimpleJSON::success(
                        "Command executed successfully",
                        movec_.getCurrentMode(req.mapping),
                        static_cast<int>(movec_.getExecutionState(req.mapping)));
                }
                return SimpleJSON::success(
                    "Command queued",
                    movec_.getCurrentMode(req.mapping),
                    static_cast<int>(movec_.getExecutionState(req.mapping))
                );
            } else {
                return SimpleJSON::error("MoveC execution failed");
            }
        } catch (const std::exception& e) {
            return SimpleJSON::error(e.what());
        }
    }

    std::string handleJointVelocity(const std::string& body) {
        try {
            auto req = RequestParser::parseJointVelocity(body);

            // 单次发送（兼容旧行为）
            if (req.duration_ms <= 0) {
                if (!joint_velocity_.execute(req.values, req.mapping)) {
                    return SimpleJSON::error("JointVelocity execution failed");
                }
                return SimpleJSON::success(
                    "Command queued",
                    joint_velocity_.getCurrentMode(req.mapping),
                    static_cast<int>(joint_velocity_.getExecutionState(req.mapping))
                );
            }

            // 先进行模式预热：在切模窗口内重试，直到真实进入 JointVelocity
            {
                const auto warmup_deadline =
                    std::chrono::steady_clock::now() + std::chrono::milliseconds(500);
                bool mode_ready = false;
                while (std::chrono::steady_clock::now() < warmup_deadline) {
                    if (!joint_velocity_.execute(req.values, req.mapping)) {
                        return SimpleJSON::error("JointVelocity warmup execution failed");
                    }
                    if (joint_velocity_.getCurrentMode(req.mapping) == "JointVelocity") {
                        mode_ready = true;
                        break;
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds(5));
                }
                if (!mode_ready) {
                    return SimpleJSON::error("JointVelocity mode not ready within warmup timeout");
                }
            }

            // 流式发送（与 example_velocity_control 相同思路）
            auto start = std::chrono::steady_clock::now();
            int sends = 0;
            while (std::chrono::duration_cast<std::chrono::milliseconds>(
                       std::chrono::steady_clock::now() - start).count() < req.duration_ms) {
                if (!joint_velocity_.execute(req.values, req.mapping)) {
                    return SimpleJSON::error("JointVelocity stream execution failed");
                }
                sends++;
                std::this_thread::sleep_for(std::chrono::milliseconds(req.interval_ms));
            }

            if (req.auto_stop) {
                std::vector<double> zero(req.values.size(), 0.0);
                joint_velocity_.execute(zero, req.mapping);
            }

            return SimpleJSON::success(
                "JointVelocity stream completed: " + std::to_string(sends) + " sends",
                joint_velocity_.getCurrentMode(req.mapping),
                static_cast<int>(joint_velocity_.getExecutionState(req.mapping))
            );
        } catch (const std::exception& e) {
            return SimpleJSON::error(e.what());
        }
    }

    std::string handleCartesianVelocity(const std::string& body) {
        try {
            auto req = RequestParser::parseCartesianVelocity(body);

            // 单次发送（兼容旧行为）
            if (req.duration_ms <= 0) {
                if (!cartesian_velocity_.execute(req.values, req.mapping)) {
                    return SimpleJSON::error("CartesianVelocity execution failed");
                }
                return SimpleJSON::success(
                    "Command queued",
                    cartesian_velocity_.getCurrentMode(req.mapping),
                    static_cast<int>(cartesian_velocity_.getExecutionState(req.mapping))
                );
            }

            // 先进行模式预热：在切模窗口内重试，直到真实进入 CartesianVelocity
            {
                const auto warmup_deadline =
                    std::chrono::steady_clock::now() + std::chrono::milliseconds(500);
                bool mode_ready = false;
                while (std::chrono::steady_clock::now() < warmup_deadline) {
                    if (!cartesian_velocity_.execute(req.values, req.mapping)) {
                        return SimpleJSON::error("CartesianVelocity warmup execution failed");
                    }
                    if (cartesian_velocity_.getCurrentMode(req.mapping) == "CartesianVelocity") {
                        mode_ready = true;
                        break;
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds(5));
                }
                if (!mode_ready) {
                    return SimpleJSON::error("CartesianVelocity mode not ready within warmup timeout");
                }
            }

            // 流式发送（与 example_velocity_control 相同思路）
            auto start = std::chrono::steady_clock::now();
            int sends = 0;
            while (std::chrono::duration_cast<std::chrono::milliseconds>(
                       std::chrono::steady_clock::now() - start).count() < req.duration_ms) {
                if (!cartesian_velocity_.execute(req.values, req.mapping)) {
                    return SimpleJSON::error("CartesianVelocity stream execution failed");
                }
                sends++;
                std::this_thread::sleep_for(std::chrono::milliseconds(req.interval_ms));
            }

            if (req.auto_stop) {
                std::vector<double> zero(req.values.size(), 0.0);
                cartesian_velocity_.execute(zero, req.mapping);
            }

            return SimpleJSON::success(
                "CartesianVelocity stream completed: " + std::to_string(sends) + " sends",
                cartesian_velocity_.getCurrentMode(req.mapping),
                static_cast<int>(cartesian_velocity_.getExecutionState(req.mapping))
            );
        } catch (const std::exception& e) {
            return SimpleJSON::error(e.what());
        }
    }

    std::string handleTrajectoryRecord(const std::string& body) {
        try {
            auto req = RequestParser::parseTeach(body);
            const std::string mapping = (req.mapping == "*" ? "" : req.mapping);
            bool ok = false;

            if (req.action == "start") {
                ok = trajectory_record_.startRecording(req.filename, mapping);
            } else if (req.action == "pause") {
                ok = trajectory_record_.pauseRecording(mapping);
            } else if (req.action == "resume") {
                ok = trajectory_record_.resumeRecording(mapping);
            } else if (req.action == "complete" || req.action == "stop") {
                ok = trajectory_record_.stopRecording(mapping);
            } else if (req.action == "cancel") {
                ok = trajectory_record_.cancelRecording(mapping);
            } else {
                return SimpleJSON::error("Unsupported action for trajectory_record");
            }

            if (!ok) {
                return SimpleJSON::error(trajectory_record_.getLastError());
            }

            const std::string mode_mapping = req.mapping.empty() || req.mapping == "*" ? "left_arm" : req.mapping;
            return SimpleJSON::success(
                "Command queued",
                trajectory_record_.getCurrentMode(mode_mapping),
                static_cast<int>(trajectory_record_.getExecutionState())
            );
        } catch (const std::exception& e) {
            return SimpleJSON::error(e.what());
        }
    }

    std::string handleTrajectoryReplay(const std::string& body) {
        try {
            auto req = RequestParser::parseTeach(body);
            const std::string mapping = (req.mapping == "*" ? "" : req.mapping);
            bool ok = false;

            if (req.action == "start") {
                ok = trajectory_replay_.startReplay(req.filename, mapping);
            } else if (req.action == "pause") {
                ok = trajectory_replay_.pauseReplay(mapping);
            } else if (req.action == "resume") {
                ok = trajectory_replay_.resumeReplay(mapping);
            } else if (req.action == "complete" || req.action == "stop") {
                ok = trajectory_replay_.stopReplay(mapping);
            } else if (req.action == "cancel") {
                ok = trajectory_replay_.cancelReplay(mapping);
            } else {
                return SimpleJSON::error("Unsupported action for trajectory_replay");
            }

            if (!ok) {
                return SimpleJSON::error(trajectory_replay_.getLastError());
            }

            const std::string mode_mapping = req.mapping.empty() || req.mapping == "*" ? "left_arm" : req.mapping;
            return SimpleJSON::success(
                "Command queued",
                trajectory_replay_.getCurrentMode(mode_mapping),
                static_cast<int>(trajectory_replay_.getExecutionState(mode_mapping))
            );
        } catch (const std::exception& e) {
            return SimpleJSON::error(e.what());
        }
    }

    std::string handleGripperControl(const std::string& body) {
        try {
            auto req = RequestParser::parseGripper(body);

            if (!basic_ops_.gripper_control(
                    req.position, req.mapping, req.velocity, req.effort, req.gripper_type)) {
                return SimpleJSON::error(basic_ops_.getLastError());
            }

            return SimpleJSON::success(
                "GripperControl command queued",
                basic_ops_.getCurrentMode(req.mapping),
                static_cast<int>(basic_ops_.getExecutionState(req.mapping))
            );
        } catch (const std::exception& e) {
            return SimpleJSON::error(e.what());
        }
    }

    std::string handleMotorEnable(const std::string& body) {
        try {
            auto req = RequestParser::parseMotorSwitch(body);
            if (!basic_ops_.enable_motors(req.mapping, req.mode)) {
                return SimpleJSON::error(basic_ops_.getLastError());
            }
            return SimpleJSON::success(
                "MotorEnable command queued",
                basic_ops_.getCurrentMode(req.mapping),
                static_cast<int>(basic_ops_.getExecutionState(req.mapping))
            );
        } catch (const std::exception& e) {
            return SimpleJSON::error(e.what());
        }
    }

    std::string handleMotorDisable(const std::string& body) {
        try {
            auto req = RequestParser::parseMotorSwitch(body);
            if (!basic_ops_.disable_motors(req.mapping, req.mode)) {
                return SimpleJSON::error(basic_ops_.getLastError());
            }
            return SimpleJSON::success(
                "MotorDisable command queued",
                basic_ops_.getCurrentMode(req.mapping),
                static_cast<int>(basic_ops_.getExecutionState(req.mapping))
            );
        } catch (const std::exception& e) {
            return SimpleJSON::error(e.what());
        }
    }
};

// 全局服务器实例
std::unique_ptr<SimpleHTTPServer> g_server;

void signalHandler(int sig) {
    if (sig == SIGINT) {
        std::cout << "\n\nShutting down..." << std::endl;
        if (g_server) {
            g_server->stop();
        }
        arm_controller::IPCLifecycle::shutdown();
        exit(0);
    }
}

int main(int /*argc*/, char** /*argv*/) {
    std::cout << "===============================================================\n"
              << "ARM Controller HTTP Server\n"
              << "===============================================================\n\n";

    // 初始化 IPC
    if (!arm_controller::IPCLifecycle::initialize()) {
        std::cerr << "❌ IPC initialization failed\n";
        return 1;
    }
    std::cout << "✅ IPC initialized\n\n";

    // 设置信号处理
    signal(SIGINT, signalHandler);

    // 启动 HTTP 服务器
    g_server = std::make_unique<SimpleHTTPServer>(8080);
    if (!g_server->start()) {
        std::cerr << "❌ Failed to start HTTP server\n";
        arm_controller::IPCLifecycle::shutdown();
        return 1;
    }

    std::cout << "🚀 Server running on http://127.0.0.1:8080\n";
    std::cout << "   ===== Available Endpoints =====\n";
    std::cout << "   POST /movej                 - Joint space point-to-point motion\n";
    std::cout << "   POST /movel                 - Cartesian linear motion\n";
    std::cout << "   POST /movec                 - Cartesian circular motion\n";
    std::cout << "   POST /joint_velocity        - Joint velocity control\n";
    std::cout << "   POST /cartesian_velocity    - Cartesian velocity control\n";
    std::cout << "   POST /trajectory_record     - Trajectory record control\n";
    std::cout << "   POST /trajectory_replay     - Trajectory replay control\n";
    std::cout << "   POST /gripper_control       - Gripper open/close (IPC basic op)\n";
    std::cout << "   POST /motor_enable          - Motor enable (IPC basic op)\n";
    std::cout << "   POST /motor_disable         - Motor disable (IPC basic op)\n";
    std::cout << "   GET  /health                - Health check\n\n";
    std::cout << "Example requests:\n";
    std::cout << "  MoveJ:\n";
    std::cout << "    curl -X POST http://127.0.0.1:8080/movej \\\n";
    std::cout << "      -H 'Content-Type: application/json' \\\n";
    std::cout << "      -d '{\"positions\": [1.0, 2.0, 3.0, 4.0, 5.0, 6.0], \"mapping\": \"single_arm\"}'\n\n";
    std::cout << "  MoveL:\n";
    std::cout << "    curl -X POST http://127.0.0.1:8080/movel \\\n";
    std::cout << "      -H 'Content-Type: application/json' \\\n";
    std::cout << "      -d '{\"x\": 0.5, \"y\": 0.3, \"z\": 0.4, \"qx\": 0, \"qy\": 0, \"qz\": 0, \"qw\": 1, \"mapping\": \"left_arm\"}'\n\n";
    std::cout << "  JointVelocity:\n";
    std::cout << "    curl -X POST http://127.0.0.1:8080/joint_velocity \\\n";
    std::cout << "      -H 'Content-Type: application/json' \\\n";
    std::cout << "      -d '{\"joint_velocities\": [0.5, 0.3, -0.2, 0, 0.1, 0], \"mapping\": \"left_arm\"}'\n\n";

    // 保持运行
    while (true) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    return 0;
}
