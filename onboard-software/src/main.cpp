#include <Arduino.h>
#include "Wire.h"
#include "imu.h"
#include "encoder.h"
#include "communication.h"
const int LEFT_ENC_DT = 18;
const int LEFT_ENC_CLK = 19;
const int RIGHT_ENC_DT = 33;
const int RIGHT_ENC_CLK = 32;
Encoder left_wheel_encoder(LEFT_ENC_CLK, LEFT_ENC_DT);
Encoder right_wheel_encoder(RIGHT_ENC_CLK, RIGHT_ENC_DT);
IMU imu;

uint8_t peer[6] = {0xEC, 0x64, 0xC9, 0x85, 0x6F, 0x84};
Communication socket;
Packet in_;
Packet out_;
// Required Components
// - top level robot
// - motor class
// - controller classes (pid, mpc, lqr)

// Control Loops
// - 1 core for handling ESP-NOW communication
// - - stuff to send from the robot
// - - - setpoint, encoder, PID, etc.
// - - stuff to send from transceiver to robot
// - - - trim value (offset to modify setpoint), turn instructions, forward/back commands
// - 1 core for handling balancing + wheel sync
// - - - design high level policy/control system (agnostic of the controller)

volatile int count = 0;

void handleIncoming(const esp_now_recv_info_t *esp_now_info, const Packet &pkt)
{
    if (pkt.type == Message::Command)
    {
        // parse pkt.payload into your command enum/struct
    }
    else if (pkt.type == Message::Telemetry)
    {
        count++;
        Serial.println("got message");
    }
}

void setup()
{
    Wire.begin();
    Serial.begin(115200);
    imu.init();
    left_wheel_encoder.init();
    right_wheel_encoder.init();
    socket.init();
    socket.addPeer(peer);
}

void loop()
{
    imu.calculateAngles();
    // Serial.print("R: ");
    // Serial.print(imu.get_roll());
    // Serial.print(" | P: ");
    // Serial.print(imu.get_pitch());
    // Serial.print(" | Y: ");
    // Serial.println(imu.get_yaw());

    out_.type = Message::Telemetry;
    Telemetry t{
        .pitch = imu.get_pitch(),
        .left_enc = left_wheel_encoder.read(),
        .right_enc = right_wheel_encoder.read(),
    };
    memcpy(out_.payload, &t, sizeof(t));
    out_.len = sizeof(t);
    out_.pitch = imu.get_pitch();
    socket.send(peer, out_);
    Serial.print("P: ");
    Serial.print(t.pitch);
    Serial.print(" | L: ");
    Serial.print(t.left_enc);
    Serial.print(" | R: ");
    Serial.println(t.right_enc);
    delay(1000);
}
