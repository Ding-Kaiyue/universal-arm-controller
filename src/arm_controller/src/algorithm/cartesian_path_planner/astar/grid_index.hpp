#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cstddef>
#include <cmath>
#include <functional>

namespace arm_controller::algorithm::cartesian_path_planner {

constexpr double kPi = 3.14159265358979323846;

struct GridIndex {
    int x{0};
    int y{0};
    int z{0};

    bool operator==(const GridIndex& other) const {
        return x == other.x && y == other.y && z == other.z;
    }
};

struct GridIndexHash {
    std::size_t operator()(const GridIndex& idx) const noexcept{
        std::size_t h1 = std::hash<int>{}(idx.x);
        std::size_t h2 = std::hash<int>{}(idx.y);
        std::size_t h3 = std::hash<int>{}(idx.z);
        return h1 ^ (h2 << 1) ^ (h3 << 2);
    }
};

struct PoseGridIndex {
    int x{0};
    int y{0};
    int z{0};
    int wx{0};
    int wy{0};
    int wz{0};

    bool operator==(const PoseGridIndex& other) const {
        return x == other.x && y == other.y && z == other.z &&
               wx == other.wx && wy == other.wy && wz == other.wz;
    }
};

struct PoseGridIndexHash {
    std::size_t operator()(const PoseGridIndex& idx) const noexcept {
        std::size_t h1 = std::hash<int>{}(idx.x);
        std::size_t h2 = std::hash<int>{}(idx.y);
        std::size_t h3 = std::hash<int>{}(idx.z);
        std::size_t h4 = std::hash<int>{}(idx.wx);
        std::size_t h5 = std::hash<int>{}(idx.wy);
        std::size_t h6 = std::hash<int>{}(idx.wz);
        return h1 ^ (h2 << 1) ^ (h3 << 2) ^ (h4 << 3) ^ (h5 << 4) ^ (h6 << 5);
    }
};

inline Eigen::Vector3d gridToWorld(
    const GridIndex& idx,
    const Eigen::Vector3d& map_min,
    double resolution) {
        return map_min + resolution * 
            (Eigen::Vector3d(idx.x, idx.y, idx.z).array() + 0.5).matrix();
}

inline GridIndex worldToGrid(
    const Eigen::Vector3d& p,
    const Eigen::Vector3d& map_min,
    double resolution) {
    Eigen::Array3d a = ((p - map_min) / resolution).array().floor(); 
    return GridIndex{
        static_cast<int>(a.x()), 
        static_cast<int>(a.y()), 
        static_cast<int>(a.z())
    };
}

inline double normalizeAngleRad(double a) {
    double out = std::fmod(a + kPi, 2.0 * kPi);
    if (out < 0.0) {
        out += 2.0 * kPi;
    }
    return out - kPi;
}

inline int orientationBinCount(double bin_size_rad) {
    const double s = std::max(1e-6, bin_size_rad);
    return std::max(8, static_cast<int>(std::round((2.0 * kPi) / s)));
}

inline int wrapOrientationBin(int b, int num_bins) {
    if (num_bins <= 0) {
        return b;
    }
    int out = b % num_bins;
    if (out < 0) {
        out += num_bins;
    }
    return out;
}

inline int wrappedBinDiff(int a, int b, int num_bins) {
    int d = std::abs(a - b);
    if (num_bins > 0) {
        d = std::min(d, num_bins - d);
    }
    return d;
}

inline int angleToBin(double angle_rad, double bin_size_rad) {
    const int n = orientationBinCount(bin_size_rad);
    const double a = normalizeAngleRad(angle_rad);
    const int b = static_cast<int>(std::llround(a / bin_size_rad));
    return wrapOrientationBin(b, n);
}

inline double binToAngle(int b, double bin_size_rad) {
    const int n = orientationBinCount(bin_size_rad);
    const int bw = wrapOrientationBin(b, n);
    const int centered = (bw <= n / 2) ? bw : (bw - n);
    return normalizeAngleRad(static_cast<double>(centered) * bin_size_rad);
}

inline Eigen::Vector3i rotationToBins(
    const Eigen::Matrix3d& R,
    double bin_size_rad) {
    const Eigen::Vector3d rpy = R.eulerAngles(0, 1, 2);
    return Eigen::Vector3i(
        angleToBin(rpy.x(), bin_size_rad),
        angleToBin(rpy.y(), bin_size_rad),
        angleToBin(rpy.z(), bin_size_rad));
}

inline Eigen::Matrix3d binsToRotation(
    int wx, int wy, int wz, double bin_size_rad) {
    const double rx = binToAngle(wx, bin_size_rad);
    const double ry = binToAngle(wy, bin_size_rad);
    const double rz = binToAngle(wz, bin_size_rad);
    return Eigen::AngleAxisd(rx, Eigen::Vector3d::UnitX()).toRotationMatrix() *
           Eigen::AngleAxisd(ry, Eigen::Vector3d::UnitY()).toRotationMatrix() *
           Eigen::AngleAxisd(rz, Eigen::Vector3d::UnitZ()).toRotationMatrix();
}

inline double rotationDistanceRad(
    const Eigen::Matrix3d& Ra,
    const Eigen::Matrix3d& Rb) {
    Eigen::AngleAxisd aa(Ra.transpose() * Rb);
    return std::abs(aa.angle());
}

inline double orientationBinDistanceRad(
    const PoseGridIndex& a,
    const PoseGridIndex& b,
    double bin_size_rad) {
    const int bins = orientationBinCount(bin_size_rad);
    const double dwx =
        static_cast<double>(wrappedBinDiff(a.wx, b.wx, bins)) * bin_size_rad;
    const double dwy =
        static_cast<double>(wrappedBinDiff(a.wy, b.wy, bins)) * bin_size_rad;
    const double dwz =
        static_cast<double>(wrappedBinDiff(a.wz, b.wz, bins)) * bin_size_rad;
    return std::sqrt(dwx * dwx + dwy * dwy + dwz * dwz);
}

inline PoseGridIndex worldPoseToGrid(
    const Eigen::Vector3d& p,
    const Eigen::Matrix3d& R,
    const Eigen::Vector3d& map_min,
    double pos_resolution,
    double ori_bin_size_rad) {
    const GridIndex pi = worldToGrid(p, map_min, pos_resolution);
    const Eigen::Vector3i oi = rotationToBins(R, ori_bin_size_rad);
    return PoseGridIndex{pi.x, pi.y, pi.z, oi.x(), oi.y(), oi.z()};
}

}  // namespace arm_controller::algorithm::cartesian_path_planner
