# Excavation hardware-facing command contract.
# ROS owns sequencing and timing. Embedded owns motor-driver execution and hard stops.
# COMMAND_START means: run the ladder forward at the embedded-side default excavation speed
# after applying local interlock checks.

uint8 COMMAND_STOP=0
uint8 COMMAND_HOME=1
uint8 COMMAND_START=2
uint8 COMMAND_CLEAR_FAULT=3

uint8 command
