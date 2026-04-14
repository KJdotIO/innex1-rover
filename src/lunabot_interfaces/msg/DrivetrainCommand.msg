# Drivetrain hardware-facing command contract.
# Sent by the drivetrain bridge to the motor controllers.
# Per-wheel throttle values normalised to [-1.0, 1.0] where positive is
# forward and negative is reverse.  The bridge maps these to Sabertooth
# Packetized Serial bytes.
#
# A command with all throttles at zero is a STOP.  If no command is received
# within the watchdog period the bridge must send a stop.

uint8 COMMAND_DRIVE=0
uint8 COMMAND_STOP=1
uint8 COMMAND_CLEAR_FAULT=2

uint8 command

# Per-wheel throttle [FL, FR, RL, RR], range [-1.0, 1.0].
# Ignored when command is COMMAND_STOP or COMMAND_CLEAR_FAULT.
float32[4] wheel_throttle
