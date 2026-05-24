#include "teensy_motor_io/safety_state.hpp"

#include <cassert>

using teensy_motor_io::ControllerState;
using teensy_motor_io::FaultCode;
using teensy_motor_io::SafetyInputs;
using teensy_motor_io::SafetyStateMachine;

int main() {
    SafetyStateMachine machine;
    SafetyInputs ok;
    ok.current_limit_configured = true;
    ok.heartbeat_fresh = true;

    auto out = machine.update(ok, false);
    assert(out.state == ControllerState::kReady);
    assert(out.fault == FaultCode::kNone);
    assert(!out.outputs_enabled);

    out = machine.update(ok, true);
    assert(out.state == ControllerState::kDriving);
    assert(out.outputs_enabled);

    auto estop = ok;
    estop.estop_active = true;
    out = machine.update(estop, true);
    assert(out.state == ControllerState::kEstop);
    assert(out.fault == FaultCode::kEstop);
    assert(!out.outputs_enabled);

    auto stale = ok;
    stale.heartbeat_fresh = false;
    out = machine.update(stale, true);
    assert(out.state == ControllerState::kFault);
    assert(out.fault == FaultCode::kJetsonHeartbeatTimeout);
    assert(!out.outputs_enabled);

    auto not_limited = ok;
    not_limited.current_limit_configured = false;
    out = machine.update(not_limited, true);
    assert(out.state == ControllerState::kFault);
    assert(out.fault == FaultCode::kBadCurrentLimit);
    assert(!out.outputs_enabled);

    return 0;
}
