#include "teensy_motor_io/safety_state.hpp"

namespace teensy_motor_io {

SafetyOutput SafetyStateMachine::update(
    const SafetyInputs& inputs,
    bool drive_requested
) {
    if (inputs.estop_active) {
        output_ = {ControllerState::kEstop, FaultCode::kEstop, false};
        return output_;
    }
    if (inputs.driver_fault) {
        output_ = {ControllerState::kFault, FaultCode::kControllerOffline, false};
        return output_;
    }
    if (!inputs.current_limit_configured) {
        output_ = {ControllerState::kFault, FaultCode::kBadCurrentLimit, false};
        return output_;
    }
    if (!inputs.heartbeat_fresh) {
        output_ = {
            ControllerState::kFault,
            FaultCode::kJetsonHeartbeatTimeout,
            false,
        };
        return output_;
    }
    if (inputs.motion_inhibited) {
        output_ = {ControllerState::kReady, FaultCode::kNone, false};
        return output_;
    }

    if (output_.state == ControllerState::kFault && !inputs.reset_faults_requested) {
        output_.outputs_enabled = false;
        return output_;
    }

    output_.fault = FaultCode::kNone;
    output_.state = drive_requested ? ControllerState::kDriving : ControllerState::kReady;
    output_.outputs_enabled = drive_requested;
    return output_;
}

}  // namespace teensy_motor_io
