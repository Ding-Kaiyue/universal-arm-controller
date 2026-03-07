#ifndef __COMMAND_QUEUE_IPC_HPP__
#define __COMMAND_QUEUE_IPC_HPP__

#include "arm_controller/ipc/shm_manager.hpp"
#include "arm_controller/ipc/ipc_context.hpp"
#include "arm_controller/ipc/ipc_types.hpp"
#include <boost/date_time/posix_time/posix_time.hpp>
#include <string>
#include <vector>
#include <cstring>
#include <iostream>
#include <memory>
#include <mutex>
#include <atomic>
#include <thread>
#include <chrono>

namespace arm_controller {

struct CommandIPC {
    static constexpr size_t MAX_MODE_LEN = 64;
    static constexpr size_t MAX_MAPPING_LEN = 64;
    static constexpr size_t MAX_COMMAND_ID_LEN = 128;
    static constexpr size_t MAX_PARAMS = 100;

    char mode[MAX_MODE_LEN];
    char mapping[MAX_MAPPING_LEN];
    char command_id[MAX_COMMAND_ID_LEN];
    double parameters[MAX_PARAMS];
    size_t param_count;
    uint64_t timestamp;

    CommandIPC() : param_count(0), timestamp(0) {
        std::memset(mode, 0, MAX_MODE_LEN);
        std::memset(mapping, 0, MAX_MAPPING_LEN);
        std::memset(command_id, 0, MAX_COMMAND_ID_LEN);
        std::memset(parameters, 0, MAX_PARAMS * sizeof(double));
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
        param_count = std::min(params.size(), size_t(MAX_PARAMS));
        for (size_t i = 0; i < param_count; ++i) {
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

class CommandQueueIPC {
public:
    static CommandQueueIPC& getInstance() {
        static CommandQueueIPC instance;
        return instance;
    }

    bool initialize() {
        shutdown_.store(false, std::memory_order_release);
        return true;
    }

    void shutdown() {
        shutdown_.store(true, std::memory_order_release);

        try {
            if (shm_manager_ && shm_manager_->isValid()) {
                auto cond = shm_manager_->getCondition();
                auto mutex = shm_manager_->getMutex();
                if (cond && mutex) {
                    // ✅ 改用 try_to_lock 避免在 shutdown 时死锁
                    boost::interprocess::scoped_lock<boost::interprocess::named_mutex>
                        lock(*mutex, boost::interprocess::try_to_lock);
                    if (lock) {
                        cond->notify_all();
                    }
                }
            }
        } catch (const std::exception& e) {
            std::cerr << "CommandQueueIPC::shutdown() failed: " << e.what() << std::endl;
        }

        // ✅ CRITICAL: 清理本地的 shm_manager_ 引用
        // 这是 IPCContext 清理完全后的最后一步，确保所有 SHM 资源被释放
        shm_manager_.reset();
    }

    bool open() {
        try {
            // ✅ 重置 shutdown 标志，允许重新打开
            shutdown_.store(false, std::memory_order_release);

            shm_manager_ = std::make_shared<ipc::SharedMemoryManager>();
            if (!shm_manager_->open()) {
                std::cerr << "CommandQueueIPC::open() failed to open shared memory" << std::endl;
                return false;
            }
            return true;
        } catch (const std::exception& e) {
            std::cerr << "CommandQueueIPC::open() failed: " << e.what() << std::endl;
            return false;
        }
    }

    void push(const CommandIPC& cmd) {
        try {
            if (!shm_manager_ || !shm_manager_->isValid()) {
                if (!open()) {
                    std::cerr << "CommandQueueIPC::push() failed to access shared memory" << std::endl;
                    return;
                }
            }

            auto queue = shm_manager_->getQueue();
            auto mutex = shm_manager_->getMutex();
            auto cond = shm_manager_->getCondition();

            if (!queue || !mutex || !cond) {
                std::cerr << "CommandQueueIPC::push() invalid queue or sync primitives" << std::endl;
                return;
            }

            ipc::TrajectoryCommand new_cmd;
            new_cmd.set_mode(cmd.get_mode());
            new_cmd.set_mapping(cmd.get_mapping());
            new_cmd.set_command_id(cmd.get_command_id());
            new_cmd.set_parameters(cmd.get_parameters());

            boost::interprocess::scoped_lock<boost::interprocess::named_mutex> lock(*mutex);

            // ✅ CRITICAL: 在持有锁后再次检查 SHM 有效性
            if (!shm_manager_ || !shm_manager_->isValid()) {
                std::cerr << "CommandQueueIPC::push() SHM closed during push" << std::endl;
                return;
            }

            auto valid_queue = shm_manager_->getQueue();
            if (!valid_queue) {
                std::cerr << "CommandQueueIPC::push() queue became invalid" << std::endl;
                return;
            }

            valid_queue->push_back(new_cmd);
            cond->notify_all();  // 唤醒所有等待的consumer，让它们竞争取queue头部的命令

        } catch (const std::exception& e) {
            std::cerr << "CommandQueueIPC::push() failed: " << e.what() << std::endl;
        }
    }

    bool pop(CommandIPC& cmd, int timeout_ms = 0) {
        try {
            // ✅ 在循环外进行初始检查，但在循环内每次都重新验证
            auto start_time = std::chrono::steady_clock::now();

            while (true) {
                // ✅ CRITICAL: 在循环的每次迭代中都重新检查 SHM 有效性
                if (!shm_manager_ || !shm_manager_->isValid()) {
                    return false;  // SHM 已关闭
                }

                auto current_mutex = shm_manager_->getMutex();
                auto current_queue = shm_manager_->getQueue();
                auto current_cond = shm_manager_->getCondition();

                if (!current_queue || !current_mutex || !current_cond) {
                    return false;  // queue、mutex 或 cond 已被销毁
                }

                boost::interprocess::scoped_lock<boost::interprocess::named_mutex> lock(*current_mutex);

                // ✅ 再次验证（在持有锁后）
                if (!shm_manager_ || !shm_manager_->isValid()) {
                    return false;  // SHM 在等待锁的过程中被关闭
                }

                auto valid_queue = shm_manager_->getQueue();
                if (!valid_queue || valid_queue->empty()) {
                    // 队列空，决定是否等待
                    if (timeout_ms <= 0 && shutdown_.load(std::memory_order_acquire) == false) {
                        // 无限等待，继续轮询
                        lock.unlock();
                        std::this_thread::sleep_for(std::chrono::milliseconds(1));
                        continue;
                    } else if (timeout_ms > 0) {
                        // 有超时时间
                        auto deadline = boost::posix_time::microsec_clock::universal_time() +
                                        boost::posix_time::milliseconds(timeout_ms);
                        current_cond->timed_wait(lock, deadline);
                    } else {
                        // timeout_ms == 0 或 shutdown，直接返回
                        return false;
                    }
                }

                // 再次检查 shutdown 标志
                if (shutdown_.load(std::memory_order_acquire)) {
                    return false;
                }

                // ✅ 最后再次验证队列有效性
                if (!shm_manager_ || !shm_manager_->isValid()) {
                    return false;
                }

                auto final_queue = shm_manager_->getQueue();
                if (final_queue && !final_queue->empty()) {
                    const auto& ipc_cmd = final_queue->front();
                    cmd.set_mode(ipc_cmd.get_mode());
                    cmd.set_mapping(ipc_cmd.get_mapping());
                    cmd.set_command_id(ipc_cmd.get_command_id());
                    cmd.set_parameters(ipc_cmd.get_parameters());
                    final_queue->pop_front();
                    return true;
                }

                // 检查是否超时
                if (timeout_ms > 0) {
                    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::steady_clock::now() - start_time).count();
                    if (elapsed >= timeout_ms) {
                        return false;
                    }
                }

                // 短暂睡眠后重试
                lock.unlock();
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        } catch (const std::exception& e) {
            std::cerr << "CommandQueueIPC::pop() failed: " << e.what() << std::endl;
            return false;
        }
    }

    // 带过滤的 pop 方法：严格按队列顺序分发命令
    // timeout_ms: 总超时时间（毫秒）。设置为负数表示无限等待。
    bool popWithFilter(CommandIPC& cmd, const std::string& target_mode, int timeout_ms = -1) {
        try {
            auto start_time = std::chrono::steady_clock::now();
            const int POLL_INTERVAL_MS = 1;

            while (true) {
                // ✅ 频繁检查 shutdown 标志，响应 Ctrl-C
                if (shutdown_.load(std::memory_order_acquire)) {
                    return false;
                }

                // ✅ CRITICAL: 在循环的每次迭代中都从 IPCContext 获取 shm_manager
                // 防止主线程调用 close() 销毁对象后，我们仍在使用悬空指针
                auto shm_manager = ipc::IPCContext::getInstance().getSharedMemoryManager();
                if (!shm_manager || !shm_manager->isValid()) {
                    return false;  // SHM 已关闭，立即返回
                }

                auto current_mutex = shm_manager->getMutex();
                if (!current_mutex) {
                    return false;  // mutex 已被销毁
                }

                {
                    // ✅ 改用 try_to_lock 避免死锁：如果无法立即获取锁，就跳过这一轮
                    boost::interprocess::scoped_lock<boost::interprocess::named_mutex>
                        lock(*current_mutex, boost::interprocess::try_to_lock);

                    // 只有成功获取锁才处理
                    if (lock) {
                        // ✅ CRITICAL: 再次检查 SHM 有效性，防止 close() 导致的竞态
                        shm_manager = ipc::IPCContext::getInstance().getSharedMemoryManager();
                        if (shm_manager && shm_manager->isValid()) {
                            auto valid_queue = shm_manager->getQueue();
                            if (valid_queue && !valid_queue->empty() && valid_queue->front().get_mode() == target_mode) {
                                cmd.set_mode(valid_queue->front().get_mode());
                                cmd.set_mapping(valid_queue->front().get_mapping());
                                cmd.set_command_id(valid_queue->front().get_command_id());
                                cmd.set_parameters(valid_queue->front().get_parameters());
                                valid_queue->pop_front();
                                return true;
                            }
                        }
                    }
                }  // ← 立即释放锁（如果成功获取的话）

                // 检查是否超时（仅在 timeout_ms >= 0 时）
                if (timeout_ms >= 0) {
                    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::steady_clock::now() - start_time).count();
                    if (elapsed >= timeout_ms) {
                        return false;  // 超时，返回 false
                    }
                }

                // ✅ 简单的非阻塞睡眠（1ms），支持快速响应 Ctrl-C
                std::this_thread::sleep_for(std::chrono::milliseconds(POLL_INTERVAL_MS));
            }
        } catch (const std::exception& e) {
            std::cerr << "CommandQueueIPC::popWithFilter() failed: " << e.what() << std::endl;
            return false;
        }
    }

    bool empty() {
        try {
            if (!shm_manager_ || !shm_manager_->isValid()) {
                return true;
            }
            auto queue = shm_manager_->getQueue();
            if (!queue) return true;

            boost::interprocess::scoped_lock<boost::interprocess::named_mutex> lock(*shm_manager_->getMutex());
            return queue->empty();
        } catch (const std::exception& e) {
            std::cerr << "CommandQueueIPC::empty() failed: " << e.what() << std::endl;
            return true;
        }
    }

    size_t size() {
        try {
            if (!shm_manager_ || !shm_manager_->isValid()) {
                return 0;
            }
            auto queue = shm_manager_->getQueue();
            if (!queue) return 0;

            boost::interprocess::scoped_lock<boost::interprocess::named_mutex> lock(*shm_manager_->getMutex());
            return queue->size();
        } catch (const std::exception& e) {
            std::cerr << "CommandQueueIPC::size() failed: " << e.what() << std::endl;
            return 0;
        }
    }

    static void cleanup() {
        ipc::SharedMemoryManager::cleanup();
    }

    // 通知等待的 consumers 检查队列（在命令执行完成后调用）
    void notifyConsumers() {
        try {
            if (!shm_manager_ || !shm_manager_->isValid()) {
                return;
            }
            auto cond = shm_manager_->getCondition();
            auto mutex = shm_manager_->getMutex();
            if (cond && mutex) {
                boost::interprocess::scoped_lock<boost::interprocess::named_mutex> lock(*mutex);
                cond->notify_all();
            }
        } catch (const std::exception& e) {
            std::cerr << "CommandQueueIPC::notifyConsumers() failed: " << e.what() << std::endl;
        }
    }

    // 获取 per-mapping 的执行互斥锁，确保同一手臂的命令串行执行
    static std::mutex& getMappingExecutionMutex(const std::string& mapping) {
        static std::map<std::string, std::mutex> mapping_mutexes;
        static std::mutex map_mutex;
        std::lock_guard<std::mutex> lock(map_mutex);
        return mapping_mutexes[mapping];
    }

private:
    CommandQueueIPC() = default;
    ~CommandQueueIPC() = default;

    std::shared_ptr<ipc::SharedMemoryManager> shm_manager_;
    std::atomic<bool> shutdown_{false};
};

} // namespace arm_controller

#endif // __COMMAND_QUEUE_IPC_HPP__