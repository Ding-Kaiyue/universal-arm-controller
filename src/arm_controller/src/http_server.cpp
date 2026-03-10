#include "arm_controller/arm_controller_api.hpp"
#include "controller/movej/movej_ipc_interface.hpp"
#include "controller/movel/movel_ipc_interface.hpp"
#include "controller/movec/movec_ipc_interface.hpp"
#include "controller/joint_velocity/joint_velocity_ipc_interface.hpp"
#include "controller/cartesian_velocity/cartesian_velocity_ipc_interface.hpp"
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
    };

    struct MoveLRequest {
        double x, y, z;
        double qx, qy, qz, qw;
        std::string mapping;
    };

    struct MoveCRequest {
        std::vector<double> waypoints;
        std::string mapping;
    };

    struct VelocityRequest {
        std::vector<double> values;
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
        char buffer[8192] = {0};
        ssize_t received = recv(client_socket, buffer, sizeof(buffer) - 1, 0);

        if (received < 0) {
            close(client_socket);
            return;
        }

        buffer[received] = '\0';
        std::string request(buffer);

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

            // 执行 JointVelocity 命令
            if (joint_velocity_.execute(req.values, req.mapping)) {
                return SimpleJSON::success(
                    "Command queued",
                    joint_velocity_.getCurrentMode(req.mapping),
                    static_cast<int>(joint_velocity_.getExecutionState(req.mapping))
                );
            } else {
                return SimpleJSON::error("JointVelocity execution failed");
            }
        } catch (const std::exception& e) {
            return SimpleJSON::error(e.what());
        }
    }

    std::string handleCartesianVelocity(const std::string& body) {
        try {
            auto req = RequestParser::parseCartesianVelocity(body);

            // 执行 CartesianVelocity 命令
            if (cartesian_velocity_.execute(req.values, req.mapping)) {
                return SimpleJSON::success(
                    "Command queued",
                    cartesian_velocity_.getCurrentMode(req.mapping),
                    static_cast<int>(cartesian_velocity_.getExecutionState(req.mapping))
                );
            } else {
                return SimpleJSON::error("CartesianVelocity execution failed");
            }
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
