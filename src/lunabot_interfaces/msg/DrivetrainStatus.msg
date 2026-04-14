# Drivetrain controller-facing status contract.
# Published by the drivetrain bridge.  Consumed by the mission coordinator
# and preflight checks to determine drivetrain readiness.

std_msgs/Header header

uint8 STATE_UNINITIALISED=0
uint8 STATE_READY=1
uint8 STATE_DRIVING=2
uint8 STATE_STOPPING=3
uint8 STATE_FAULT=4
uint8 STATE_ESTOP=5

uint16 FAULT_NONE=0
uint16 FAULT_ESTOP=1
uint16 FAULT_ENCODER_STALL=2
uint16 FAULT_CONTROLLER_OFFLINE=3
uint16 FAULT_OVERCURRENT=4
uint16 FAULT_COMMAND_TIMEOUT=5

uint8  state
uint16 fault_code
bool   estop_active
bool   motion_inhibited
bool[2] controller_online
