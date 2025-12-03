#include "arm_controller/arm_controller_api.hpp"
#include "arm_controller/ipc/ipc_context.hpp"

namespace arm_controller {

bool IPCLifecycle::initialize(int argc, char** argv) {
    return ipc::IPCContext::getInstance().initialize(argc, argv);
}

void IPCLifecycle::shutdown() {
    ipc::IPCContext::getInstance().shutdown();
}

bool IPCLifecycle::isInitialized() {
    return ipc::IPCContext::getInstance().isInitialized();
}

}  // namespace arm_controller
