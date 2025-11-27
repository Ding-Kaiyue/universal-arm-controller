#ifndef __SHM_MANAGER_HPP__
#define __SHM_MANAGER_HPP__

#include "ipc_types.hpp"
#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/sync/named_mutex.hpp>
#include <boost/interprocess/sync/named_condition.hpp>
#include <boost/interprocess/containers/deque.hpp>
#include <memory>
#include <string>

namespace arm_controller::ipc {

using ManagedShmAllocator =
    boost::interprocess::allocator<TrajectoryCommand,
        boost::interprocess::managed_shared_memory::segment_manager>;

using CommandDeque =
    boost::interprocess::deque<TrajectoryCommand, ManagedShmAllocator>;

// ============================================================================
// SharedMemoryManager：负责创建、打开、管理共享内存
// 单一职责：IPC 资源的生命周期管理
// ============================================================================
class SharedMemoryManager {
public:
    // 公开常量
    static constexpr const char* SHM_NAME = "arm_controller_shm_v1";
    static constexpr const char* QUEUE_NAME = "command_queue";
    static constexpr const char* HEADER_NAME = "shm_header";
    static constexpr const char* MUTEX_NAME = "arm_controller_mutex";
    static constexpr const char* COND_NAME = "arm_controller_cond";
    static constexpr size_t SHM_SIZE = 16 * 1024 * 1024;  // 16 MB

    SharedMemoryManager() = default;
    ~SharedMemoryManager() = default;

    // 初始化共享内存（管理进程调用）
    bool initialize();

    // 打开共享内存（普通进程调用）
    bool open();

    // 获取队列对象
    CommandDeque* getQueue();

    // 获取头对象
    ShmHeader* getHeader();

    // 获取互斥量
    boost::interprocess::named_mutex* getMutex();

    // 获取条件变量
    boost::interprocess::named_condition* getCondition();

    // 清理共享内存（管理进程调用）
    static void cleanup();

    // 验证共享内存有效性
    bool isValid() const;

    // 关闭共享内存
    void close();

private:
    std::shared_ptr<boost::interprocess::managed_shared_memory> segment_;
    std::shared_ptr<boost::interprocess::named_mutex> mutex_;
    std::shared_ptr<boost::interprocess::named_condition> condition_;
    CommandDeque* queue_ = nullptr;
    ShmHeader* header_ = nullptr;
    bool initialized_ = false;
};

}  // namespace arm_controller::ipc

#endif  // __SHM_MANAGER_HPP__
