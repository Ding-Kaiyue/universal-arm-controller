#ifndef __TRAJECTORY_SEGMENTER_HPP__
#define __TRAJECTORY_SEGMENTER_HPP__

#include <vector>
#include <memory>

/**
 * @brief 轨迹分段器
 * 将长轨迹数据分段处理，便于大数据量的回放
 */
class TrajectorySegmenter {
public:
    struct Segment {
        size_t start_idx;
        size_t end_idx;
        size_t point_count;
        bool is_first = false;
    };

    /**
     * @brief 构造函数
     * @param segment_size 每个分段的最大点数
     */
    explicit TrajectorySegmenter(size_t segment_size = 100000);

    /**
     * @brief 计算分段信息
     * @param total_points 总数据点数
     * @return 返回所有分段信息
     */
    std::vector<Segment> compute_segments(size_t total_points);

    /**
     * @brief 获取当前分段大小
     */
    size_t get_segment_size() const { return segment_size_; }

    /**
     * @brief 设置分段大小
     */
    void set_segment_size(size_t size) { segment_size_ = size; }

    /**
     * @brief 获取分段总数
     */
    size_t get_segment_count() const { return segment_count_; }

private:
    size_t segment_size_;
    size_t segment_count_ = 0;
};

#endif // __TRAJECTORY_SEGMENTER_HPP__
