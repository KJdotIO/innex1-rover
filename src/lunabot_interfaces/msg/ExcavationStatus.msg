# Excavation controller-facing status contract.
# This is the typed state snapshot shared between the controller and action adapter.

std_msgs/Header header

uint8 STATE_IDLE=0
uint8 STATE_HOMING=1
uint8 STATE_READY=2
uint8 STATE_STARTING=3
uint8 STATE_EXCAVATING=4
uint8 STATE_STOPPING=5
uint8 STATE_FAULT=6

uint16 FAULT_NONE=0
uint16 FAULT_ESTOP=1
uint16 FAULT_DRIVER=2
uint16 FAULT_OVERCURRENT=3
uint16 FAULT_HOME_SWITCH_INVALID=4
uint16 FAULT_COMMAND_REJECTED=5

uint8 state
uint16 fault_code
bool estop_active
bool driver_fault
bool homed
bool motor_enabled
float32 motor_current_a
