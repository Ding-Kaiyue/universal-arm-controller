#ifndef __COMMAND_QUEUE_IPC_HPP__
#define __COMMAND_QUEUE_IPC_HPP__

#include "arm_controller/ipc/shm_manager.hpp"
#include "arm_controller/ipc/ipc_types.hpp"
#include <boost/date_time/posix_time/posix_time.hpp>
#include <string>
#include <vector>
#include <cstring>
#include <iostream>
#include <memory>

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
    static CommandQueueIPC& getInstance() {
        static CommandQueueIPC instance;
        return instance;
    }

    bool initialize() {
        try {
            shm_manager_ = std::make_shared<ipc::SharedMemoryManager>();
            if (!shm_manager_->initialize()) {
                std::cerr << "CommandQueueIPC::initialize() failed to create shared memory" << std::endl;
                return false;
            }
            return true;
        } catch (const std::exception& e) {
            std::cerr << "CommandQueueIPC::initialize() failed: " << e.what() << std::endl;
            return false;
        }
    }

    bool open() {
        try {
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

    void push(const TrajectoryCommandIPC& cmd) {
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
            queue->push_back(new_cmd);
            cond->notify_one();

        } catch (const std::exception& e) {
            std::cerr << "CommandQueueIPC::push() failed: " << e.what() << std::endl;
        }
    }

    bool pop(TrajectoryCommandIPC& cmd, int timeout_ms = 0) {
        try {
            if (!shm_manager_ || !shm_manager_->isValid()) {
                if (!open()) {
                    return false;
                }
            }

            auto queue = shm_manager_->getQueue();
            auto mutex = shm_manager_->getMutex();
            auto cond = shm_manager_->getCondition();

            if (!queue || !mutex || !cond) {
                return false;
            }

            boost::interprocess::scoped_lock<boost::interprocess::named_mutex> lock(*mutex);

            if (queue->empty()) {
                if (timeout_ms <= 0) return false;

                auto deadline = boost::posix_time::microsec_clock::universal_time() +
                    boost::posix_time::milliseconds(timeout_ms);
                if (!cond->timed_wait(lock, deadline)) {
                    return false;
                }
            }

            if (!queue->empty()) {
                const auto& ipc_cmd = queue->front();
                cmd.set_mode(ipc_cmd.get_mode());
                cmd.set_mapping(ipc_cmd.get_mapping());
                cmd.set_command_id(ipc_cmd.get_command_id());
                cmd.set_parameters(ipc_cmd.get_parameters());
                queue->pop_front();
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

private:
    CommandQueueIPC() = default;
    ~CommandQueueIPC() = default;

    std::shared_ptr<ipc::SharedMemoryManager> shm_manager_;
};

} // namespace arm_controller

#endif // __COMMAND_QUEUE_IPC_HPP__