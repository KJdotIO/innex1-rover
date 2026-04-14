# Drivetrain hardware-facing telemetry contract.
# Published by the drivetrain bridge at the control loop rate.
# All encoder-derived values are per-wheel, ordered [FL, FR, RL, RR].

std_msgs/Header header

# Encoder-derived feedback per wheel.
float32[4] wheel_velocity_rps     # Wheel angular velocity (revolutions/sec)
int32[4]   encoder_ticks           # Raw cumulative encoder tick count

# Controller status per Sabertooth (index 0 = front pair, 1 = rear pair).
bool[2] controller_online         # True if UART heartbeat received

# Safety state (mirrors /safety/motion_inhibit for convenience).
bool estop_active
bool motion_inhibited

# Fault information.
uint16 fault_code

uint16 FAULT_NONE=0
uint16 FAULT_ESTOP=1
uint16 FAULT_ENCODER_STALL=2
uint16 FAULT_CONTROLLER_OFFLINE=3
uint16 FAULT_OVERCURRENT=4
uint16 FAULT_COMMAND_TIMEOUT=5
