#include <pybind11/pybind11.h>
﻿#include <pybind11/stl.h>

﻿#include "awscr/brain.hpp"
﻿#include "awscr/dynamics_3dof.hpp"
﻿#include "awscr/mission_fsm.hpp"
﻿#include "awscr/types.hpp"

namespace py = pybind11;
using namespace awscr;

PYBIND11_MODULE(pyawscr, m) {
  m.doc() = "Python bindings for AWSCR core brain";

  py::enum_<FsmMode>(m, "FsmMode")
      .value("IDLE", FsmMode::IDLE)
      .value("PLANNING", FsmMode::PLANNING)
      .value("EXECUTING_COVERAGE", FsmMode::EXECUTING_COVERAGE)
      .value("EXECUTING_INTERCEPT", FsmMode::EXECUTING_INTERCEPT)
      .value("DOCKING", FsmMode::DOCKING)
      .value("ERROR", FsmMode::ERROR);

  py::class_<VesselParams>(m, "VesselParams")
      .def(py::init<>())
      .def_readwrite("m", &VesselParams::m)
      .def_readwrite("x_G", &VesselParams::x_G)
      .def_readwrite("I_z", &VesselParams::I_z)
      .def_readwrite("X_u", &VesselParams::X_u)
      .def_readwrite("Y_v", &VesselParams::Y_v)
      .def_readwrite("N_r", &VesselParams::N_r)
      .def_readwrite("X_uu", &VesselParams::X_uu)
      .def_readwrite("Y_vv", &VesselParams::Y_vv)
      .def_readwrite("N_rr", &VesselParams::N_rr)
      .def_readwrite("X_du", &VesselParams::X_du)
      .def_readwrite("Y_dv", &VesselParams::Y_dv)
      .def_readwrite("N_dr", &VesselParams::N_dr)
      .def_readwrite("thruster_y", &VesselParams::thruster_y);

  py::class_<State3Dof>(m, "State3Dof")
      .def(py::init<>())
      .def_readwrite("x", &State3Dof::x)
      .def_readwrite("y", &State3Dof::y)
      .def_readwrite("psi", &State3Dof::psi)
      .def_readwrite("u", &State3Dof::u)
      .def_readwrite("v", &State3Dof::v)
      .def_readwrite("r", &State3Dof::r);

  py::class_<Input3Dof>(m, "Input3Dof")
      .def(py::init<>())
      .def_readwrite("T_left", &Input3Dof::T_left)
      .def_readwrite("T_right", &Input3Dof::T_right)
      .def_readwrite("u_c", &Input3Dof::u_c)
      .def_readwrite("v_c", &Input3Dof::v_c)
      .def_readwrite("tau_wave_X", &Input3Dof::tau_wave_X)
      .def_readwrite("tau_wave_Y", &Input3Dof::tau_wave_Y)
      .def_readwrite("tau_wave_N", &Input3Dof::tau_wave_N);

  py::class_<Dynamics3Dof>(m, "Dynamics3Dof")
      .def(py::init<const VesselParams&>())
      .def("derivative", &Dynamics3Dof::derivative);

  py::class_<ImuSample>(m, "ImuSample")
      .def(py::init<>())
      .def_readwrite("t", &ImuSample::t)
      .def_readwrite("ang_vel_b", &ImuSample::ang_vel_b)
      .def_readwrite("lin_acc_b", &ImuSample::lin_acc_b);

  py::class_<GpsFix>(m, "GpsFix")
      .def(py::init<>())
      .def_readwrite("t", &GpsFix::t)
      .def_readwrite("lat_deg", &GpsFix::lat_deg)
      .def_readwrite("lon_deg", &GpsFix::lon_deg)
      .def_readwrite("alt_m", &GpsFix::alt_m)
      .def_readwrite("valid", &GpsFix::valid);

  py::class_<MagSample>(m, "MagSample")
      .def(py::init<>())
      .def_readwrite("t", &MagSample::t)
      .def_readwrite("mag_b", &MagSample::mag_b);

  py::class_<MotorThrustCmd>(m, "MotorThrustCmd")
      .def(py::init<>())
      .def_readwrite("t", &MotorThrustCmd::t)
      .def_readwrite("thrust_left_N", &MotorThrustCmd::thrust_left_N)
      .def_readwrite("thrust_right_N", &MotorThrustCmd::thrust_right_N);

  py::class_<Telemetry>(m, "Telemetry")
      .def(py::init<>())
      .def_readwrite("t", &Telemetry::t)
      .def_readwrite("lat_deg", &Telemetry::lat_deg)
      .def_readwrite("lon_deg", &Telemetry::lon_deg)
      .def_readwrite("heading_deg", &Telemetry::heading_deg)
      .def_readwrite("speed_mps", &Telemetry::speed_mps)
      .def_readwrite("mode", &Telemetry::mode);

  py::enum_<CommandType>(m, "CommandType")
      .value("NONE", CommandType::NONE)
      .value("START_MISSION", CommandType::START_MISSION)
      .value("STOP", CommandType::STOP)
      .value("DOCK", CommandType::DOCK)
      .value("RESET", CommandType::RESET);

  py::class_<Command>(m, "Command")
      .def(py::init<>())
      .def_readwrite("type", &Command::type);

  py::class_<Brain>(m, "Brain")
      .def(py::init<const VesselParams&>())
      .def("set_reference_origin", &Brain::set_reference_origin)
      .def("set_coverage_polygon", &Brain::set_coverage_polygon)
      .def("feed_imu", &Brain::feed_imu)
      .def("feed_gps", &Brain::feed_gps)
      .def("feed_mag", &Brain::feed_mag)
      .def("handle_command", &Brain::handle_command)
      .def("step", &Brain::step)
      .def("get_telemetry", &Brain::get_telemetry);
}

