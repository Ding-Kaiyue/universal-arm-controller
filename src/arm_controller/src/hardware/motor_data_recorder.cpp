#include "arm_controller/hardware/motor_data_recorder.hpp"
#include <cstdio>
#include <cstring>
#include <errno.h>

MotorDataRecorder::MotorDataRecorder(const std::string& output_file,
                                    const std::string& mapping,
                                    size_t buffer_size)
    : output_file_(output_file), mapping_(mapping), fd_(-1), mmap_buffer_(nullptr),
      total_buffer_size_(buffer_size), buffer_pos_(0) {

    // 打开或创建文件 - 需要 O_RDWR 来支持 MAP_SHARED
    fd_ = open(output_file.c_str(), O_CREAT | O_RDWR | O_TRUNC, 0644);
    if (fd_ < 0) {
        perror("❎ Failed to open file for mmap");
        fprintf(stderr, "❎ Cannot open file: %s (errno: %d)\n", output_file.c_str(), errno);
        return;
    }
    fprintf(stderr, "✅ File opened: %s (fd: %d)\n", output_file.c_str(), fd_);

    // 预分配文件大小（快速）
    if (lseek(fd_, buffer_size - 1, SEEK_SET) < 0) {
        perror("❎ lseek failed");
        close(fd_);
        fd_ = -1;
        return;
    }
    if (write(fd_, "", 1) < 0) {
        perror("❎ write failed");
        close(fd_);
        fd_ = -1;
        return;
    }
    fprintf(stderr, "✅ File pre-allocated: %zu bytes\n", buffer_size);

    // 映射到内存
    // 使用 O_RDWR 打开文件，现在可以使用 MAP_SHARED
    mmap_buffer_ = (char*)mmap(nullptr, buffer_size, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, 0);
    if (mmap_buffer_ == MAP_FAILED) {
        perror("❎ mmap failed");
        fprintf(stderr, "❎ mmap failed (errno: %d)\n", errno);
        close(fd_);
        fd_ = -1;
        mmap_buffer_ = nullptr;
        return;
    }
    fprintf(stderr, "✅ mmap successful (MAP_SHARED), buffer: %p\n", (void*)mmap_buffer_);

    // 写入 CSV 表头
    const char header[] = "timestamp,interface,motor_id,position,velocity,effort,temperature,enable_flag,motor_mode\n";
    size_t header_len = strlen(header);
    memcpy(mmap_buffer_, header, header_len);
    buffer_pos_ = header_len;

    start_time_ = std::chrono::steady_clock::now();
}

MotorDataRecorder::MotorDataRecorder(const std::string& output_file,
                                    const std::string& mapping,
                                    const std::vector<uint32_t>& motor_ids,
                                    size_t buffer_size)
    : output_file_(output_file), mapping_(mapping), fd_(-1), mmap_buffer_(nullptr),
      total_buffer_size_(buffer_size), buffer_pos_(0), expected_motor_ids_(motor_ids) {

    // 打开或创建文件 - 需要 O_RDWR 来支持 MAP_SHARED
    fd_ = open(output_file.c_str(), O_CREAT | O_RDWR | O_TRUNC, 0644);
    if (fd_ < 0) {
        perror("❎ Failed to open file for mmap");
        fprintf(stderr, "❎ Cannot open file: %s (errno: %d)\n", output_file.c_str(), errno);
        return;
    }
    fprintf(stderr, "✅ File opened: %s (fd: %d)\n", output_file.c_str(), fd_);

    // 预分配文件大小（快速）
    if (lseek(fd_, buffer_size - 1, SEEK_SET) < 0) {
        perror("❎ lseek failed");
        close(fd_);
        fd_ = -1;
        return;
    }
    if (write(fd_, "", 1) < 0) {
        perror("❎ write failed");
        close(fd_);
        fd_ = -1;
        return;
    }
    fprintf(stderr, "✅ File pre-allocated: %zu bytes\n", buffer_size);

    // 映射到内存
    // 使用 O_RDWR 打开文件，现在可以使用 MAP_SHARED
    mmap_buffer_ = (char*)mmap(nullptr, buffer_size, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, 0);
    if (mmap_buffer_ == MAP_FAILED) {
        perror("❎ mmap failed");
        fprintf(stderr, "❎ mmap failed (errno: %d)\n", errno);
        close(fd_);
        fd_ = -1;
        mmap_buffer_ = nullptr;
        return;
    }
    fprintf(stderr, "✅ mmap successful (MAP_SHARED), buffer: %p\n", (void*)mmap_buffer_);

    // 写入 CSV 表头
    const char header[] = "timestamp,interface,motor_id,position,velocity,effort,temperature,enable_flag,motor_mode\n";
    size_t header_len = strlen(header);
    memcpy(mmap_buffer_, header, header_len);
    buffer_pos_ = header_len;

    start_time_ = std::chrono::steady_clock::now();
}

MotorDataRecorder::~MotorDataRecorder() {
    if (mmap_buffer_ != nullptr && mmap_buffer_ != MAP_FAILED) {
        // 同步到磁盘
        msync(mmap_buffer_, buffer_pos_, MS_SYNC);
        munmap(mmap_buffer_, total_buffer_size_);
    }
    if (fd_ >= 0) {
        close(fd_);
    }
}

void MotorDataRecorder::sync_to_disk() {
    if (mmap_buffer_ != nullptr && mmap_buffer_ != MAP_FAILED) {
        msync(mmap_buffer_, buffer_pos_, MS_SYNC);
    }
}

void MotorDataRecorder::on_motor_status_update(
    const std::string& interface,
    uint32_t motor_id,
    const hardware_driver::motor_driver::Motor_Status& status) {

    // ✅ 检查是否暂停 - 暂停时不写入任何数据
    if (is_paused()) {
        return;
    }

    if (mmap_buffer_ == nullptr || mmap_buffer_ == MAP_FAILED) {
        fprintf(stderr, "❌ mmap_buffer is null or failed\n");
        return;
    }

    auto now = std::chrono::steady_clock::now();
    double elapsed = std::chrono::duration<double>(now - start_time_).count();

    // 检查缓冲区是否有足够空间（预留 512 字节）
    if (buffer_pos_ + 512 >= total_buffer_size_) {
        // 缓冲区满，同步到磁盘并继续（实际生产环境应该告警）
        msync(mmap_buffer_, buffer_pos_, MS_ASYNC);
        return;
    }

    // ✅ 超快速格式化数据到缓冲区
    // 使用 snprintf 避免缓冲区溢出，性能仍然很好（<10μs）
    int written = snprintf(
        &mmap_buffer_[buffer_pos_],
        512,
        "%.6f,%s,%u,%.6f,%.6f,%.6f,%.1f,%d,%d\n",
        elapsed,
        interface.c_str(),
        motor_id,
        status.position,
        status.velocity,
        status.effort,
        static_cast<int>(status.temperature) / 10.0,
        static_cast<int>(status.enable_flag),
        static_cast<int>(status.motor_mode)
    );

    if (written > 0) {
        buffer_pos_ += written;

        // ✅ 单数据模式：记录所有电机各一条数据后自动暂停
        if (single_record_mode_ && !expected_motor_ids_.empty()) {
            // 标记此电机已记录
            recorded_motor_ids_.insert(motor_id);

            // 检查是否所有电机都已记录
            bool all_recorded = true;
            for (uint32_t expected_id : expected_motor_ids_) {
                if (recorded_motor_ids_.find(expected_id) == recorded_motor_ids_.end()) {
                    all_recorded = false;
                    break;
                }
            }

            // 如果所有电机都已记录，暂停
            if (all_recorded) {
                is_paused_[mapping_] = true;
            }
        }
    }
}
