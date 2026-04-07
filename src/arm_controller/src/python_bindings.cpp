#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "arm_controller/arm_controller_api.hpp"
#include "arm_controller/ipc/controller_state_manager.hpp"
#include "controller/basic_ops/basic_ops_ipc_interface.hpp"
#include "controller/movej/movej_ipc_interface.hpp"
#include "controller/joint_velocity/joint_velocity_ipc_interface.hpp"
#include "controller/cartesian_velocity/cartesian_velocity_ipc_interface.hpp"
#include "controller/command_streaming/command_streaming_ipc_interface.hpp"

namespace py = pybind11;

PYBIND11_MODULE(arm_controller_py, m) {
    m.doc() = "Python bindings for universal arm controller IPC interfaces";

    py::enum_<arm_controller::ipc::ExecutionState>(m, "ExecutionState")
        .value("IDLE", arm_controller::ipc::ExecutionState::IDLE)
        .value("PENDING", arm_controller::ipc::ExecutionState::PENDING)
        .value("EXECUTING", arm_controller::ipc::ExecutionState::EXECUTING)
        .value("SUCCESS", arm_controller::ipc::ExecutionState::SUCCESS)
        .value("FAILED", arm_controller::ipc::ExecutionState::FAILED)
        .export_values();

    py::class_<arm_controller::IPCLifecycle>(m, "IPCLifecycle")
        .def_static("initialize", []() {
            return arm_controller::IPCLifecycle::initialize(0, nullptr);
        })
        .def_static("initialize_as_consumer", []() {
            return arm_controller::IPCLifecycle::initializeAsConsumer(0, nullptr);
        })
        .def_static("shutdown", &arm_controller::IPCLifecycle::shutdown)
        .def_static("is_initialized", &arm_controller::IPCLifecycle::isInitialized);

    py::class_<arm_controller::joint_velocity::JointVelocityIPCInterface>(
        m, "JointVelocityIPCInterface")
        .def(py::init<>())
        .def("execute",
             &arm_controller::joint_velocity::JointVelocityIPCInterface::execute,
             py::arg("joint_velocities"),
             py::arg("mapping"))
        .def("get_current_mode",
             &arm_controller::joint_velocity::JointVelocityIPCInterface::getCurrentMode,
             py::arg("mapping"))
        .def("get_execution_state",
             &arm_controller::joint_velocity::JointVelocityIPCInterface::getExecutionState,
             py::arg("mapping"))
        .def("get_last_error",
             &arm_controller::joint_velocity::JointVelocityIPCInterface::getLastError);

    py::class_<arm_controller::cartesian_velocity::CartesianVelocityIPCInterface>(
        m, "CartesianVelocityIPCInterface")
        .def(py::init<>())
        .def("execute",
             &arm_controller::cartesian_velocity::CartesianVelocityIPCInterface::execute,
             py::arg("cartesian_velocities"),
             py::arg("mapping"))
        .def("get_current_mode",
             &arm_controller::cartesian_velocity::CartesianVelocityIPCInterface::getCurrentMode,
             py::arg("mapping"))
        .def("get_execution_state",
             &arm_controller::cartesian_velocity::CartesianVelocityIPCInterface::getExecutionState,
             py::arg("mapping"))
        .def("get_last_error",
             &arm_controller::cartesian_velocity::CartesianVelocityIPCInterface::getLastError);

    py::class_<arm_controller::command_streaming::CommandStreamingIPCInterface>(
        m, "CommandStreamingIPCInterface")
        .def(py::init<>())
        .def("execute",
             &arm_controller::command_streaming::CommandStreamingIPCInterface::execute,
             py::arg("joint_velocities"),
             py::arg("mapping"))
        .def("get_current_mode",
             &arm_controller::command_streaming::CommandStreamingIPCInterface::getCurrentMode,
             py::arg("mapping"))
        .def("get_execution_state",
             &arm_controller::command_streaming::CommandStreamingIPCInterface::getExecutionState,
             py::arg("mapping"))
        .def("get_last_error",
             &arm_controller::command_streaming::CommandStreamingIPCInterface::getLastError);

    py::class_<arm_controller::movej::MoveJIPCInterface>(
        m, "MoveJIPCInterface")
        .def(py::init<>())
        .def("execute",
             &arm_controller::movej::MoveJIPCInterface::execute,
             py::arg("joint_positions"),
             py::arg("mapping"))
        .def("get_current_mode",
             &arm_controller::movej::MoveJIPCInterface::getCurrentMode,
             py::arg("mapping"))
        .def("get_execution_state",
             &arm_controller::movej::MoveJIPCInterface::getExecutionState,
             py::arg("mapping"))
        .def("get_last_error",
             &arm_controller::movej::MoveJIPCInterface::getLastError);

    py::class_<arm_controller::basic_ops::BasicOpsIPCInterface>(
        m, "BasicOpsIPCInterface")
        .def(py::init<>())
        .def("gripper_control",
             &arm_controller::basic_ops::BasicOpsIPCInterface::gripper_control,
             py::arg("position"),
             py::arg("mapping"),
             py::arg("velocity"),
             py::arg("effort"),
             py::arg("gripper_type") = -1)
        .def("enable_motors",
             &arm_controller::basic_ops::BasicOpsIPCInterface::enable_motors,
             py::arg("mapping"),
             py::arg("mode"))
        .def("disable_motors",
             &arm_controller::basic_ops::BasicOpsIPCInterface::disable_motors,
             py::arg("mapping"),
             py::arg("mode"))
        .def("get_current_mode",
             &arm_controller::basic_ops::BasicOpsIPCInterface::getCurrentMode,
             py::arg("mapping"))
        .def("get_execution_state",
             &arm_controller::basic_ops::BasicOpsIPCInterface::getExecutionState,
             py::arg("mapping"))
        .def("get_last_error",
             &arm_controller::basic_ops::BasicOpsIPCInterface::getLastError);
}
