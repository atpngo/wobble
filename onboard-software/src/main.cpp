#include <Arduino.h>
#include "imu.h"
#include "encoder.h"
// IMU imu;
const int LEFT_ENC_DT = 18;
const int LEFT_ENC_CLK = 19;
const int RIGHT_ENC_DT = 33;
const int RIGHT_ENC_CLK = 32;
Encoder left_wheel_encoder(LEFT_ENC_CLK, LEFT_ENC_DT);
Encoder right_wheel_encoder(RIGHT_ENC_CLK, RIGHT_ENC_DT);
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
    Serial.begin(115200);
    left_wheel_encoder.init();
    right_wheel_encoder.init();
}

void loop()
{
    Serial.print("L: ");
    Serial.print(left_wheel_encoder.read());
    Serial.print(" | R: ");
    Serial.println(right_wheel_encoder.read());
}
