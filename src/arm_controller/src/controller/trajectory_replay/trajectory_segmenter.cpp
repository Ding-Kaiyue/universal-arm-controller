#include "trajectory_segmenter.hpp"
#include <algorithm>

TrajectorySegmenter::TrajectorySegmenter(size_t segment_size)
    : segment_size_(segment_size) {}

std::vector<TrajectorySegmenter::Segment> TrajectorySegmenter::compute_segments(size_t total_points) {
    std::vector<Segment> segments;

    if (total_points == 0) {
        return segments;
    }

    // 计算分段数
    segment_count_ = (total_points + segment_size_ - 1) / segment_size_;

    for (size_t seg = 0; seg < segment_count_; ++seg) {
        Segment segment;
        segment.start_idx = seg * segment_size_;
        segment.end_idx = std::min(segment.start_idx + segment_size_, total_points);
        segment.point_count = segment.end_idx - segment.start_idx;
        segment.is_first = (seg == 0);

        segments.push_back(segment);
    }

    return segments;
}
