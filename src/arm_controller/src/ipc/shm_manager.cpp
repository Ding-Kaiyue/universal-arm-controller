#include "arm_controller/ipc/shm_manager.hpp"
#include <boost/interprocess/shared_memory_object.hpp>
#include <iostream>

namespace arm_controller::ipc {

bool SharedMemoryManager::initialize() {
    try {
        // 清理旧的共享内存（如果存在）
        try {
            boost::interprocess::shared_memory_object::remove(SHM_NAME);
        } catch (...) {}

        // 创建新的共享内存段
        segment_ = std::make_shared<boost::interprocess::managed_shared_memory>(
            boost::interprocess::create_only,
            SHM_NAME,
            SHM_SIZE);

        // 初始化 header
        header_ = segment_->construct<ShmHeader>(HEADER_NAME)();
        if (!header_) {
            std::cerr << "Failed to construct ShmHeader" << std::endl;
            return false;
        }
        header_->version = IPC_VERSION;
        header_->segment_size = SHM_SIZE;

        // 创建命令队列
        const ManagedShmAllocator alloc(segment_->get_segment_manager());
        queue_ = segment_->construct<CommandDeque>(QUEUE_NAME)(alloc);
        if (!queue_) {
            std::cerr << "Failed to construct CommandDeque" << std::endl;
            return false;
        }

        // 创建互斥量和条件变量
        mutex_ = std::make_shared<boost::interprocess::named_mutex>(
            boost::interprocess::open_or_create,
            MUTEX_NAME);

        condition_ = std::make_shared<boost::interprocess::named_condition>(
            boost::interprocess::open_or_create,
            COND_NAME);

        initialized_ = true;
        std::cout << "✅ SharedMemoryManager initialized successfully" << std::endl;
        return true;

    } catch (const std::exception& e) {
        std::cerr << "❌ SharedMemoryManager::initialize() failed: " << e.what() << std::endl;
        return false;
    }
}

bool SharedMemoryManager::open() {
    try {
        // 打开现有的共享内存段
        segment_ = std::make_shared<boost::interprocess::managed_shared_memory>(
            boost::interprocess::open_only,
            SHM_NAME);

        // 查找 header
        header_ = segment_->find<ShmHeader>(HEADER_NAME).first;
        if (!header_) {
            std::cerr << "Failed to find ShmHeader" << std::endl;
            return false;
        }

        if (!header_->isValid()) {
            std::cerr << "ShmHeader is invalid (version or magic mismatch)" << std::endl;
            return false;
        }

        // 查找命令队列
        queue_ = segment_->find<CommandDeque>(QUEUE_NAME).first;
        if (!queue_) {
            std::cerr << "Failed to find CommandDeque" << std::endl;
            return false;
        }

        // 打开互斥量和条件变量
        mutex_ = std::make_shared<boost::interprocess::named_mutex>(
            boost::interprocess::open_only,
            MUTEX_NAME);

        condition_ = std::make_shared<boost::interprocess::named_condition>(
            boost::interprocess::open_only,
            COND_NAME);

        initialized_ = true;
        std::cout << "✅ SharedMemoryManager opened successfully" << std::endl;
        return true;

    } catch (const std::exception& e) {
        std::cerr << "❌ SharedMemoryManager::open() failed: " << e.what() << std::endl;
        return false;
    }
}

CommandDeque* SharedMemoryManager::getQueue() {
    if (!initialized_ || !queue_) {
        std::cerr << "❌ SharedMemoryManager not properly initialized" << std::endl;
        return nullptr;
    }
    return queue_;
}

ShmHeader* SharedMemoryManager::getHeader() {
    if (!initialized_ || !header_) {
        std::cerr << "❌ SharedMemoryManager not properly initialized" << std::endl;
        return nullptr;
    }
    return header_;
}

boost::interprocess::named_mutex* SharedMemoryManager::getMutex() {
    if (!initialized_ || !mutex_) {
        std::cerr << "❌ SharedMemoryManager not properly initialized" << std::endl;
        return nullptr;
    }
    return mutex_.get();
}

boost::interprocess::named_condition* SharedMemoryManager::getCondition() {
    if (!initialized_ || !condition_) {
        std::cerr << "❌ SharedMemoryManager not properly initialized" << std::endl;
        return nullptr;
    }
    return condition_.get();
}

bool SharedMemoryManager::isValid() const {
    return initialized_ && header_ && header_->isValid() && queue_;
}

void SharedMemoryManager::close() {
    queue_ = nullptr;
    header_ = nullptr;
    segment_.reset();
    mutex_.reset();
    condition_.reset();
    initialized_ = false;
}

void SharedMemoryManager::cleanup() {
    try {
        boost::interprocess::shared_memory_object::remove(SHM_NAME);
        boost::interprocess::named_mutex::remove(MUTEX_NAME);
        boost::interprocess::named_condition::remove(COND_NAME);
        std::cout << "✅ SharedMemoryManager cleanup completed" << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "⚠️  SharedMemoryManager::cleanup() warning: " << e.what() << std::endl;
    }
}

}  // namespace arm_controller::ipc