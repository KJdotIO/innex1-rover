## Code Test Function:
- Move actuator one direction
- Stop
- Move actuator the other direction
- Stop
- Repeat


The PWM value controls actuator speed.
The DIR value controls actuator direction.

> Code was run in the STM32CubeIDE.
> To stop the actuator disconnect the power supply (Stopping the program only stops the debug session in the IDE).

## Hardware Used:
- Laptop
- STM32 Nucleo F401RE
- Cytron MDD10A Rev 2.0 motor driver
- Linear actuator
- DC power supply
- Jumper wires
- Terminal block / connectors

## Connections:
- Nucleo GND is connected to Cytron GND
- Nucleo D13 is connected to Cytron PWM1
- Nucleo D12 is connected to Cytron DIR1
- Power supply + is connected to Cytron B+
- Power supply - is connected to Cytron B-
- Actuator is connected to M1A and M1B
- No loose copper strands are touching
- The actuator is clear and safe to move

> Refer to Circuit Diagram
