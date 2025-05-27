#include <Arduino.h>
#include "Wire.h"
#include "imu.h"
#include "encoder.h"
#include "communication.h"
#include "motor.h"
// #include <ArduinoEigen.h>

//
// Pin Declarations
//
// Left motor
const int LEFT_ENC_DT = 18;
const int LEFT_ENC_CLK = 19;
const int LEFT_MOTOR_A = 4;
const int LEFT_MOTOR_B = 0;
const int LEFT_MOTOR_ENABLE = 16;
// Right motor
const int RIGHT_ENC_DT = 33;
const int RIGHT_ENC_CLK = 32;
const int RIGHT_MOTOR_A = 25;
const int RIGHT_MOTOR_B = 26;
const int RIGHT_MOTOR_ENABLE = 27;

//
// Robot objects
//
Encoder left_wheel_encoder(LEFT_ENC_CLK, LEFT_ENC_DT);
Encoder right_wheel_encoder(RIGHT_ENC_CLK, RIGHT_ENC_DT);
IMU imu;
Motor left_motor(LEFT_MOTOR_A, LEFT_MOTOR_B, LEFT_MOTOR_ENABLE);
Motor right_motor(RIGHT_MOTOR_A, RIGHT_MOTOR_B, RIGHT_MOTOR_ENABLE);

//
// Communication between robot and transceiver
//
Communication socket;
Packet in_;
Packet out_;
Command data_;
uint8_t peer[6] = {0xEC,
                   0x64,
                   0xC9,
                   0x85,
                   0x6F,
                   0x84};

// Required Components
// - top level robot
// - motor class
// - controller classes (pid, mpc, lqr)

// Control Loops
// - 1 core for handling ESP-NOW communication
// - - stuff to send from transceiver to robot
// - - - trim value (offset to modify setpoint), turn instructions, forward/back commands
// - 1 core for handling balancing + wheel sync
// - - - design high level policy/control system (agnostic of the controller)

// Balance task:
// - Drive the main wheel based on the pitch provided by the IMU
// void BalanceTask(void *parameter)
// {
//     while (1)
//     {
//         robot.balance();
//     }
// }

// // Follow task:
// // - Drive the secondary wheel to follow the main one based on differences in encoder value
// void FollowTask(void *parameters)
// {
//     while (1)
//     {
//         robot.secondaryMotor.follow(robot.mainMotor);
//     }
// }

void handleIncoming(const esp_now_recv_info_t *esp_now_info, const uint8_t *data, int len)
{
    memcpy(&in_, data, len);
    if (in_.type == Message::Command)
    {
        memcpy(&data_, in_.payload, in_.len);
        // Serial.print("L: ");
        // Serial.print(data_.left_wheel_power);
        // Serial.print(" | R: ");
        // Serial.println(data_.right_wheel_power);
        left_motor.spin(data_.left_wheel_power);
        right_motor.spin(data_.right_wheel_power);
    }
}

void setup()
{
    Wire.begin();
    Serial.begin(115200);
    right_motor.set_type(MotorType::MAIN);
    left_motor.set_type(MotorType::SECONDARY);
    imu.init();
    left_wheel_encoder.init();
    right_wheel_encoder.init();
    socket.init();
    socket.addPeer(peer);
    socket.onReceive(handleIncoming);
}

void loop()
{
    imu.calculateAngles();

    out_.type = Message::Telemetry;
    Telemetry t{
        .pitch = imu.get_pitch(),
        .left_enc = left_wheel_encoder.read(),
        .right_enc = right_wheel_encoder.read(),
    };
    memcpy(out_.payload, &t, sizeof(t));
    out_.len = sizeof(t);
    socket.send(peer, out_);

    delay(100);
}
