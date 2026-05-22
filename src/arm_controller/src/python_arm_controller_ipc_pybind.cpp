#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "arm_controller/arm_controller_api.hpp"
#include "arm_controller/ipc/controller_state_manager.hpp"
#if ARM_CONTROLLER_ENABLE_MOTION_CONTROLLERS
#include "controller/movej/movej_ipc_interface.hpp"
#include "controller/movel/movel_ipc_interface.hpp"
#include "controller/movec/movec_ipc_interface.hpp"
#include "controller/reactive_task/reactive_task_ipc_interface.hpp"
#endif
#if ARM_CONTROLLER_ENABLE_VELOCITY_CONTROLLERS
#include "controller/joint_velocity/joint_velocity_ipc_interface.hpp"
#include "controller/cartesian_velocity/cartesian_velocity_ipc_interface.hpp"
#include "controller/mink_servo/mink_servo_ipc_interface.hpp"
#endif
#if ARM_CONTROLLER_ENABLE_TEACH_CONTROLLERS
#include "controller/trajectory_record/trajectory_record_ipc_interface.hpp"
#include "controller/trajectory_replay/trajectory_replay_ipc_interface.hpp"
#endif
#include "controller/basic_ops/basic_ops_ipc_interface.hpp"

namespace py = pybind11;

PYBIND11_MODULE(arm_controller_ipc, m) {
    m.doc() = "Python IPC producer bindings for universal arm controller";

    py::enum_<arm_controller::ipc::ExecutionState>(m, "ExecutionState")
        .value("IDLE", arm_controller::ipc::ExecutionState::IDLE)
        .value("PENDING", arm_controller::ipc::ExecutionState::PENDING)
        .value("EXECUTING", arm_controller::ipc::ExecutionState::EXECUTING)
        .value("SUCCESS", arm_controller::ipc::ExecutionState::SUCCESS)
        .value("FAILED", arm_controller::ipc::ExecutionState::FAILED)
        .export_values();

    m.def("initialize_producer", []() {
        return arm_controller::IPCLifecycle::initialize(0, nullptr);
    }, "Attach as IPC Producer(Participant)");

    m.def("shutdown", []() {
        arm_controller::IPCLifecycle::shutdown();
    }, "Shutdown local IPC context");

    m.def("is_initialized", []() {
        return arm_controller::IPCLifecycle::isInitialized();
    });

#if ARM_CONTROLLER_ENABLE_MOTION_CONTROLLERS
    py::class_<arm_controller::movej::MoveJIPCInterface>(m, "MoveJ")
        .def(py::init<>())
        .def("execute", &arm_controller::movej::MoveJIPCInterface::execute,
             py::arg("joint_positions"), py::arg("mapping"))
        .def("get_current_mode", &arm_controller::movej::MoveJIPCInterface::getCurrentMode,
             py::arg("mapping"))
        .def("get_execution_state", &arm_controller::movej::MoveJIPCInterface::getExecutionState,
             py::arg("mapping"))
        .def("get_last_error", &arm_controller::movej::MoveJIPCInterface::getLastError);

    py::class_<arm_controller::movel::MoveLIPCInterface>(m, "MoveL")
        .def(py::init<>())
        .def("execute", &arm_controller::movel::MoveLIPCInterface::execute,
             py::arg("x"), py::arg("y"), py::arg("z"),
             py::arg("qx"), py::arg("qy"), py::arg("qz"), py::arg("qw"),
             py::arg("mapping"))
        .def("get_current_mode", &arm_controller::movel::MoveLIPCInterface::getCurrentMode,
             py::arg("mapping"))
        .def("get_execution_state", &arm_controller::movel::MoveLIPCInterface::getExecutionState,
             py::arg("mapping"))
        .def("get_last_error", &arm_controller::movel::MoveLIPCInterface::getLastError);

    py::class_<arm_controller::movec::MoveCIPCInterface>(m, "MoveC")
        .def(py::init<>())
        .def("execute", &arm_controller::movec::MoveCIPCInterface::execute,
             py::arg("waypoints"), py::arg("mapping"))
        .def("get_current_mode", &arm_controller::movec::MoveCIPCInterface::getCurrentMode,
             py::arg("mapping"))
        .def("get_execution_state", &arm_controller::movec::MoveCIPCInterface::getExecutionState,
             py::arg("mapping"))
        .def("get_last_error", &arm_controller::movec::MoveCIPCInterface::getLastError);

    py::class_<arm_controller::reactive_task::ReactiveTaskIPCInterface>(m, "ReactiveTask")
        .def(py::init<>())
        .def("execute", &arm_controller::reactive_task::ReactiveTaskIPCInterface::execute,
             py::arg("target_pose"), py::arg("mapping"))
        .def("get_current_mode", &arm_controller::reactive_task::ReactiveTaskIPCInterface::getCurrentMode,
             py::arg("mapping"))
        .def("get_execution_state", &arm_controller::reactive_task::ReactiveTaskIPCInterface::getExecutionState,
             py::arg("mapping"))
        .def("get_last_error", &arm_controller::reactive_task::ReactiveTaskIPCInterface::getLastError);
#endif

#if ARM_CONTROLLER_ENABLE_VELOCITY_CONTROLLERS
    py::class_<arm_controller::joint_velocity::JointVelocityIPCInterface>(m, "JointVelocity")
        .def(py::init<>())
        .def("execute", &arm_controller::joint_velocity::JointVelocityIPCInterface::execute,
             py::arg("joint_velocities"), py::arg("mapping"))
        .def("get_current_mode", &arm_controller::joint_velocity::JointVelocityIPCInterface::getCurrentMode,
             py::arg("mapping"))
        .def("get_execution_state", &arm_controller::joint_velocity::JointVelocityIPCInterface::getExecutionState,
             py::arg("mapping"))
        .def("get_last_error", &arm_controller::joint_velocity::JointVelocityIPCInterface::getLastError);

    py::class_<arm_controller::cartesian_velocity::CartesianVelocityIPCInterface>(m, "CartesianVelocity")
        .def(py::init<>())
        .def("execute", &arm_controller::cartesian_velocity::CartesianVelocityIPCInterface::execute,
             py::arg("cartesian_velocities"), py::arg("mapping"))
        .def("get_current_mode", &arm_controller::cartesian_velocity::CartesianVelocityIPCInterface::getCurrentMode,
             py::arg("mapping"))
        .def("get_execution_state", &arm_controller::cartesian_velocity::CartesianVelocityIPCInterface::getExecutionState,
             py::arg("mapping"))
        .def("get_last_error", &arm_controller::cartesian_velocity::CartesianVelocityIPCInterface::getLastError);

    py::class_<arm_controller::mink_servo::MinkServoIPCInterface>(m, "MinkServo")
        .def(py::init<>())
        .def("execute", &arm_controller::mink_servo::MinkServoIPCInterface::execute,
             py::arg("target_pose"), py::arg("mapping"))
        .def("get_current_mode", &arm_controller::mink_servo::MinkServoIPCInterface::getCurrentMode,
             py::arg("mapping"))
        .def("get_execution_state", &arm_controller::mink_servo::MinkServoIPCInterface::getExecutionState,
             py::arg("mapping"))
        .def("get_last_error", &arm_controller::mink_servo::MinkServoIPCInterface::getLastError);
#endif

#if ARM_CONTROLLER_ENABLE_TEACH_CONTROLLERS
    py::class_<arm_controller::trajectory_record::TrajectoryRecordIPCInterface>(m, "TrajectoryRecord")
        .def(py::init<>())
        .def("start_recording", &arm_controller::trajectory_record::TrajectoryRecordIPCInterface::startRecording,
             py::arg("file_name"), py::arg("mapping") = "")
        .def("pause_recording", &arm_controller::trajectory_record::TrajectoryRecordIPCInterface::pauseRecording,
             py::arg("mapping") = "")
        .def("resume_recording", &arm_controller::trajectory_record::TrajectoryRecordIPCInterface::resumeRecording,
             py::arg("mapping") = "")
        .def("stop_recording", &arm_controller::trajectory_record::TrajectoryRecordIPCInterface::stopRecording,
             py::arg("mapping") = "")
        .def("cancel_recording", &arm_controller::trajectory_record::TrajectoryRecordIPCInterface::cancelRecording,
             py::arg("mapping") = "")
        .def("get_current_mode", &arm_controller::trajectory_record::TrajectoryRecordIPCInterface::getCurrentMode,
             py::arg("mapping"))
        .def("get_execution_state", &arm_controller::trajectory_record::TrajectoryRecordIPCInterface::getExecutionState)
        .def("is_recording", &arm_controller::trajectory_record::TrajectoryRecordIPCInterface::isRecording)
        .def("get_last_error", &arm_controller::trajectory_record::TrajectoryRecordIPCInterface::getLastError);

    py::class_<arm_controller::trajectory_replay::TrajectoryReplayIPCInterface>(m, "TrajectoryReplay")
        .def(py::init<>())
        .def("start_replay", &arm_controller::trajectory_replay::TrajectoryReplayIPCInterface::startReplay,
             py::arg("file_name"), py::arg("mapping") = "")
        .def("pause_replay", &arm_controller::trajectory_replay::TrajectoryReplayIPCInterface::pauseReplay,
             py::arg("mapping") = "")
        .def("resume_replay", &arm_controller::trajectory_replay::TrajectoryReplayIPCInterface::resumeReplay,
             py::arg("mapping") = "")
        .def("stop_replay", &arm_controller::trajectory_replay::TrajectoryReplayIPCInterface::stopReplay,
             py::arg("mapping") = "")
        .def("cancel_replay", &arm_controller::trajectory_replay::TrajectoryReplayIPCInterface::cancelReplay,
             py::arg("mapping") = "")
        .def("get_current_mode", &arm_controller::trajectory_replay::TrajectoryReplayIPCInterface::getCurrentMode,
             py::arg("mapping"))
        .def("get_execution_state", &arm_controller::trajectory_replay::TrajectoryReplayIPCInterface::getExecutionState,
             py::arg("mapping"))
        .def("is_replaying", &arm_controller::trajectory_replay::TrajectoryReplayIPCInterface::isReplaying,
             py::arg("mapping"))
        .def("get_last_error", &arm_controller::trajectory_replay::TrajectoryReplayIPCInterface::getLastError);
#endif

    py::class_<arm_controller::basic_ops::BasicOpsIPCInterface>(m, "BasicOps")
        .def(py::init<>())
        .def("gripper_control", &arm_controller::basic_ops::BasicOpsIPCInterface::gripper_control,
             py::arg("position"), py::arg("mapping"), py::arg("velocity"), py::arg("effort"), py::arg("gripper_type") = -1)
        .def("enable_motors", &arm_controller::basic_ops::BasicOpsIPCInterface::enable_motors,
             py::arg("mapping"), py::arg("mode"))
        .def("disable_motors", &arm_controller::basic_ops::BasicOpsIPCInterface::disable_motors,
             py::arg("mapping"), py::arg("mode"))
        .def("get_current_mode", &arm_controller::basic_ops::BasicOpsIPCInterface::getCurrentMode,
             py::arg("mapping"))
        .def("get_execution_state", &arm_controller::basic_ops::BasicOpsIPCInterface::getExecutionState,
             py::arg("mapping"))
        .def("get_last_error", &arm_controller::basic_ops::BasicOpsIPCInterface::getLastError);
}
