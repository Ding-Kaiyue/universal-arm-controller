#pragma once

#include <chrono>

namespace arm_controller::algorithm::cartesian_path_planner::utils {

class ScopedTimer {
public:
    ScopedTimer() : t0_(std::chrono::steady_clock::now()) {}

    double elapsedSec() const {
        const auto dt = std::chrono::steady_clock::now() - t0_;
        return std::chrono::duration<double>(dt).count();
    }

private:
    std::chrono::steady_clock::time_point t0_;
};

}  // namespace arm_controller::algorithm::cartesian_path_planner::utils
