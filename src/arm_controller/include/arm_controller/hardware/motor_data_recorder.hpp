#pragma once

#include <string>
#include <vector>
#include <unordered_map>
#include <thread>
#include <atomic>
#include <mutex>
#include <fstream>
#include <sstream>
#include <array>

#include "hardware_driver/driver/motor_driver_interface.hpp"

class MotorDataRecorder : public hardware_driver::motor_driver::MotorStatusObserver
{
public:

    using MotorStatus =
        hardware_driver::motor_driver::Motor_Status;

    static constexpr size_t DOF = 6;

    explicit MotorDataRecorder(const std::string& output_file);

    ~MotorDataRecorder();

    void start();

    void stop();

    void register_interface(
        const std::string& interface,
        const std::vector<uint32_t>& motor_ids);

    void on_motor_status_update(
        const std::string& interface,
        uint32_t motor_id,
        const MotorStatus& status);

private:

    struct Snapshot
    {
        double timestamp;

        std::array<float,DOF> pos;
        std::array<float,DOF> vel;
        std::array<float,DOF> eff;
    };

    struct InterfaceBuffer
    {
        // Protects all mutable members below for producer/consumer concurrency.
        mutable std::mutex mutex;

        std::vector<uint32_t> motor_ids;

        std::unordered_map<uint32_t,size_t> motor_index;

        std::array<MotorStatus,DOF> latest_state;

        std::array<bool,DOF> updated;

        std::vector<Snapshot> ring;

        size_t head = 0;
        size_t tail = 0;

        size_t capacity = 4096;
    };

private:

    void writer_loop();

private:

    std::string output_file_;

    std::ofstream file_;

    std::unordered_map<std::string,InterfaceBuffer> interfaces_;

    std::mutex config_mutex_;

    std::thread writer_thread_;

    std::atomic<bool> running_{false};

    std::chrono::steady_clock::time_point start_time_;
};
