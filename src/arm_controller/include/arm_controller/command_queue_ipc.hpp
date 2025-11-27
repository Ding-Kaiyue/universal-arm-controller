#ifndef __COMMAND_QUEUE_IPC_HPP__
#define __COMMAND_QUEUE_IPC_HPP__

#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/allocators/allocator.hpp>
#include <boost/interprocess/containers/deque.hpp>
#include <boost/interprocess/sync/named_mutex.hpp>
#include <boost/interprocess/sync/named_condition.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <string>
#include <vector>
#include <cstring>
#include <iostream>

namespace arm_controller {

struct TrajectoryCommandIPC {
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

    TrajectoryCommandIPC() : param_count(0), timestamp(0) {
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
    static constexpr const char* SEGMENT_NAME = "arm_controller_ipc_segment";
    static constexpr const char* QUEUE_NAME = "arm_controller_command_queue";
    static constexpr const char* MUTEX_NAME = "arm_controller_queue_mutex";
    static constexpr const char* COND_NAME = "arm_controller_queue_cond";
    static constexpr size_t SEGMENT_SIZE = 65536;

    using ShmAllocator = boost::interprocess::allocator<TrajectoryCommandIPC,
        boost::interprocess::managed_shared_memory::segment_manager>;
    using CommandDeque = boost::interprocess::deque<TrajectoryCommandIPC, ShmAllocator>;

    static CommandQueueIPC& getInstance() {
        static CommandQueueIPC instance;
        return instance;
    }

    bool initialize() {
        try {
            boost::interprocess::shared_memory_object::remove(SEGMENT_NAME);
        } catch (...) {}

        try {
            segment_ = std::make_shared<boost::interprocess::managed_shared_memory>(
                boost::interprocess::create_only, SEGMENT_NAME, SEGMENT_SIZE);

            const ShmAllocator alloc(segment_->get_segment_manager());
            queue_ = segment_->construct<CommandDeque>(QUEUE_NAME)(alloc);

            mutex_ = std::make_shared<boost::interprocess::named_mutex>(
                boost::interprocess::open_or_create, MUTEX_NAME);
            cond_ = std::make_shared<boost::interprocess::named_condition>(
                boost::interprocess::open_or_create, COND_NAME);

            return true;
        } catch (const std::exception& e) {
            std::cerr << "CommandQueueIPC::initialize() failed: " << e.what() << std::endl;
            return false;
        }
    }

    bool open() {
        try {
            segment_ = std::make_shared<boost::interprocess::managed_shared_memory>(
                boost::interprocess::open_only, SEGMENT_NAME);

            queue_ = segment_->find<CommandDeque>(QUEUE_NAME).first;
            if (!queue_) {
                std::cerr << "CommandQueueIPC: queue not found" << std::endl;
                return false;
            }

            mutex_ = std::make_shared<boost::interprocess::named_mutex>(
                boost::interprocess::open_only, MUTEX_NAME);
            cond_ = std::make_shared<boost::interprocess::named_condition>(
                boost::interprocess::open_only, COND_NAME);

            return true;
        } catch (const std::exception& e) {
            std::cerr << "CommandQueueIPC::open() failed: " << e.what() << std::endl;
            return false;
        }
    }

    void push(const TrajectoryCommandIPC& cmd) {
        try {
            boost::interprocess::scoped_lock<boost::interprocess::named_mutex> lock(*mutex_);
            if (queue_) {
                queue_->push_back(cmd);
                cond_->notify_one();
            }
        } catch (const std::exception& e) {
            std::cerr << "CommandQueueIPC::push() failed: " << e.what() << std::endl;
        }
    }

    bool pop(TrajectoryCommandIPC& cmd, int timeout_ms = 0) {
        try {
            boost::interprocess::scoped_lock<boost::interprocess::named_mutex> lock(*mutex_);

            if (!queue_ || queue_->empty()) {
                if (timeout_ms <= 0) return false;

                auto deadline = boost::posix_time::microsec_clock::universal_time() +
                    boost::posix_time::milliseconds(timeout_ms);
                if (!cond_->timed_wait(lock, deadline)) {
                    return false;
                }
            }

            if (queue_ && !queue_->empty()) {
                cmd = queue_->front();
                queue_->pop_front();
                return true;
            }
            return false;
        } catch (const std::exception& e) {
            std::cerr << "CommandQueueIPC::pop() failed: " << e.what() << std::endl;
            return false;
        }
    }

    bool empty() {
        try {
            boost::interprocess::scoped_lock<boost::interprocess::named_mutex> lock(*mutex_);
            return !queue_ || queue_->empty();
        } catch (const std::exception& e) {
            std::cerr << "CommandQueueIPC::empty() failed: " << e.what() << std::endl;
            return true;
        }
    }

    size_t size() {
        try {
            boost::interprocess::scoped_lock<boost::interprocess::named_mutex> lock(*mutex_);
            return queue_ ? queue_->size() : 0;
        } catch (const std::exception& e) {
            std::cerr << "CommandQueueIPC::size() failed: " << e.what() << std::endl;
            return 0;
        }
    }

    static void cleanup() {
        try {
            boost::interprocess::shared_memory_object::remove(SEGMENT_NAME);
            boost::interprocess::named_mutex::remove(MUTEX_NAME);
            boost::interprocess::named_condition::remove(COND_NAME);
        } catch (...) {}
    }

private:
    CommandQueueIPC() : queue_(nullptr) {}
    ~CommandQueueIPC() = default;

    std::shared_ptr<boost::interprocess::managed_shared_memory> segment_;
    CommandDeque* queue_;
    std::shared_ptr<boost::interprocess::named_mutex> mutex_;
    std::shared_ptr<boost::interprocess::named_condition> cond_;
};

} // namespace arm_controller

#endif // __COMMAND_QUEUE_IPC_HPP__