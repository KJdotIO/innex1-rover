# Excavation hardware-facing telemetry contract.
# These fields are the minimum required to run a deterministic excavation controller.

std_msgs/Header header

uint16 FAULT_NONE=0
uint16 FAULT_ESTOP=1
uint16 FAULT_DRIVER=2
uint16 FAULT_OVERCURRENT=3
uint16 FAULT_HOME_SWITCH_INVALID=4
uint16 FAULT_COMMAND_REJECTED=5

bool estop_active
bool driver_fault
bool home_switch
bool motor_enabled
float32 motor_current_a
uint16 fault_code
