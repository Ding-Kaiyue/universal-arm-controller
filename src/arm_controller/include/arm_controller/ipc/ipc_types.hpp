#ifndef __IPC_TYPES_HPP__
#define __IPC_TYPES_HPP__

#include <cstdint>
#include <cstring>
#include <vector>
#include <string>
#include <algorithm>

namespace arm_controller::ipc {

// ============================================================================
// IPC 版本与常量
// ============================================================================
constexpr uint32_t IPC_VERSION = 1;
constexpr size_t MAX_JOINTS = 16;
constexpr size_t MAX_COMMAND_ID_LEN = 128;

// ============================================================================
// 数据结构 1: 共享内存头
// ============================================================================
struct alignas(64) ShmHeader {
    uint32_t version;           // 版本号，用于兼容性检查
    uint32_t magic;             // 幻数（0xDEADBEEF），检测损坏
    uint64_t created_timestamp; // 创建时间戳（ns）
    uint32_t segment_size;      // segment 大小
    uint32_t reserved;          // 预留字段

    ShmHeader()
        : version(IPC_VERSION),
          magic(0xDEADBEEF),
          created_timestamp(0),
          segment_size(0),
          reserved(0) {}

    bool isValid() const {
        return magic == 0xDEADBEEF && version == IPC_VERSION;
    }
};

// ============================================================================
// 数据结构 2: 轨迹命令（核心业务数据）
// ============================================================================
struct alignas(16) TrajectoryCommand {
    static constexpr size_t MAX_MODE_LEN = 32;
    static constexpr size_t MAX_MAPPING_LEN = 32;
    static constexpr size_t MAX_COMMAND_ID_LEN = 128;

    // 控制字段
    uint64_t seq;                              // 序号，用于检测丢包
    uint64_t timestamp_ns;                     // 生产者写入时的时间戳
    uint32_t producer_id;                      // 生产者ID（安全检查）
    uint32_t command_type;                     // 命令类型（0=MoveJ, 1=MoveL, 2=MoveC）

    // 命令内容
    char mode[MAX_MODE_LEN];                   // "MoveJ", "MoveL", "MoveC"
    char mapping[MAX_MAPPING_LEN];             // 目标映射（"left_arm", "right_arm"）
    char command_id[MAX_COMMAND_ID_LEN];       // 唯一命令ID（用于追踪）

    // 参数（根据 command_type 解释不同含义）
    int32_t param_count;                       // 参数数量（MoveJ: 关节数; MoveL/MoveC: 坐标点数）
    double parameters[MAX_JOINTS];             // 参数值（MoveJ: 关节位置; MoveL: x,y,z,qx,qy,qz,qw; MoveC: 轨迹点坐标）
    double velocities[MAX_JOINTS];             // 速度（预留，后续使用）
    double efforts[MAX_JOINTS];                // 力矩/力（预留，后续使用）

    // 完整性检查
    uint32_t crc32;                            // CRC校验（防止传输过程损坏）

    TrajectoryCommand()
        : seq(0),
          timestamp_ns(0),
          producer_id(0),
          command_type(0),
          param_count(0),
          crc32(0) {
        std::memset(mode, 0, MAX_MODE_LEN);
        std::memset(mapping, 0, MAX_MAPPING_LEN);
        std::memset(command_id, 0, MAX_COMMAND_ID_LEN);
        std::memset(parameters, 0, MAX_JOINTS * sizeof(double));
        std::memset(velocities, 0, MAX_JOINTS * sizeof(double));
        std::memset(efforts, 0, MAX_JOINTS * sizeof(double));
    }

    // 计算CRC（简化实现，可集成第三方库）
    void calculateCrc() {
        crc32 = 0;  // 实际使用crc库计算
        // crc32 = boost::crc_32_type()(this, offsetof(TrajectoryCommand, crc32));
    }

    bool validateCrc() const {
        // return crc32 == boost::crc_32_type()(this, offsetof(TrajectoryCommand, crc32));
        return true;  // 简化：暂不检查
    }

    bool isValid() const {
        return !validateCrc() || (param_count > 0 && param_count <= static_cast<int32_t>(MAX_JOINTS));
    }

    void set_mode(const std::string& m) {
        std::strncpy(mode, m.c_str(), MAX_MODE_LEN - 1);
        mode[MAX_MODE_LEN - 1] = '\0';
    }

    void set_mapping(const std::string& map) {
        std::strncpy(mapping, map.c_str(), MAX_MAPPING_LEN - 1);
        mapping[MAX_MAPPING_LEN - 1] = '\0';
    }

    void set_command_id(const std::string& id) {
        std::strncpy(command_id, id.c_str(), MAX_COMMAND_ID_LEN - 1);
        command_id[MAX_COMMAND_ID_LEN - 1] = '\0';
    }

    void set_parameters(const std::vector<double>& params) {
        param_count = std::min(params.size(), size_t(MAX_JOINTS));
        for (int32_t i = 0; i < param_count; ++i) {
            parameters[i] = params[i];
        }
    }

    std::string get_mode() const { return std::string(mode); }
    std::string get_mapping() const { return std::string(mapping); }
    std::string get_command_id() const { return std::string(command_id); }

    std::vector<double> get_parameters() const {
        return std::vector<double>(parameters, parameters + param_count);
    }
};

// ============================================================================
// 数据结构 3: 心跳（消费者状态反馈）
// ============================================================================
struct alignas(16) Heartbeat {
    uint64_t seq;                  // 对应已处理的最后一个 TrajectoryCommand 的 seq
    uint64_t timestamp_ns;         // 心跳时间戳
    uint32_t consumer_id;          // 消费者ID
    int32_t status;                // 0=idle, 1=executing, -1=error
    char error_msg[64];            // 最后一条错误信息

    Heartbeat()
        : seq(0),
          timestamp_ns(0),
          consumer_id(0),
          status(0) {
        std::memset(error_msg, 0, 64);
    }
};

// ============================================================================
// 数据结构 4: 队列元数据（与 ring-buffer 配合使用）
// ============================================================================
struct QueueMetadata {
    uint32_t capacity;             // 队列容量
    uint32_t head;                 // 读指针
    uint32_t tail;                 // 写指针
    uint32_t count;                // 当前元素个数
    uint64_t dropped_count;        // 丢弃的命令计数（溢出时）
};

}  // namespace arm_controller::ipc

#endif  // __IPC_TYPES_HPP__
