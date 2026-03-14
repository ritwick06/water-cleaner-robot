#pragma once

#include "types.hpp"

namespace awscr {

enum class CommandType { NONE, START_MISSION, STOP, DOCK, RESET };

struct Command {
  CommandType type{CommandType::NONE};
};

class MissionFsm {
 public:
  MissionFsm() = default;

  void handle_command(const Command& cmd) {
    switch (cmd.type) {
      case CommandType::START_MISSION:
        mode_ = FsmMode::PLANNING;
        break;
      case CommandType::STOP:
        mode_ = FsmMode::IDLE;
        break;
      case CommandType::DOCK:
        mode_ = FsmMode::DOCKING;
        break;
      case CommandType::RESET:
        mode_ = FsmMode::IDLE;
        break;
      case CommandType::NONE:
      default:
        break;
    }
  }

  void set_has_plan(bool ok) {
    if (ok && mode_ == FsmMode::PLANNING) {
      mode_ = FsmMode::EXECUTING_COVERAGE;
    }
  }

  FsmMode mode() const { return mode_; }

 private:
  FsmMode mode_{FsmMode::IDLE};
};

}  // namespace awscr

