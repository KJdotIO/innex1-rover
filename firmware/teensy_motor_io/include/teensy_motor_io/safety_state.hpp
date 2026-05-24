#pragma once

#include <cstdint>

namespace teensy_motor_io {

enum class ControllerState : std::uint8_t {
    kUninitialised = 0,
    kReady = 1,
    kDriving = 2,
    kFault = 4,
    kEstop = 5,
};

enum class FaultCode : std::uint16_t {
    kNone = 0,
    kEstop = 1,
    kEncoderStall = 2,
    kControllerOffline = 3,
    kOvercurrent = 4,
    kCommandTimeout = 5,
    kBadCurrentLimit = 6,
    kJetsonHeartbeatTimeout = 7,
};

struct SafetyInputs {
    bool estop_active{false};
    bool motion_inhibited{false};
    bool driver_fault{false};
    bool current_limit_configured{false};
    bool heartbeat_fresh{false};
    bool reset_faults_requested{false};
};

struct SafetyOutput {
    ControllerState state{ControllerState::kUninitialised};
    FaultCode fault{FaultCode::kNone};
    bool outputs_enabled{false};
};

class SafetyStateMachine {
public:
    SafetyOutput update(const SafetyInputs& inputs, bool drive_requested);
    SafetyOutput output() const { return output_; }

private:
    SafetyOutput output_{};
};

}  // namespace teensy_motor_io
