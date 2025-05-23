#include <Arduino.h>

// Required Components
// - top level robot
// - motor class
// - imu class
// - encoder class? TODO: check if the encoder class we wrote for the controller is cross-platform
// - controller classes (pid, mpc, lqr)

// Control Loops
// - 1 core for handling ESP-NOW communication
// - - stuff to send from the robot
// - - - setpoint, encoder, PID, etc.
// - - stuff to send from transceiver to robot
// - - - trim value (offset to modify setpoint), turn instructions, forward/back commands
// - 1 core for handling balancing + wheel sync
// - - - design high level policy/control system (agnostic of the controller)

void setup()
{
}

void loop()
{
}
