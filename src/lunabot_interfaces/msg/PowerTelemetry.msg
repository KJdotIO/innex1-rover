# Rover power telemetry. Early hardware runs may publish this from a manual
# bridge; later runs should replace that source with the chosen meter/sensor.

std_msgs/Header header

uint8 STATE_UNAVAILABLE=0
uint8 STATE_OK=1
uint8 STATE_LOW_WARNING=2
uint8 STATE_LOW_CRITICAL=3

string profile
string source
uint8 state
float32 bus_voltage_v
float32 bus_current_a
float32 energy_wh
float32 warning_voltage_v
float32 critical_voltage_v
bool low_voltage_warning
bool low_voltage_critical
