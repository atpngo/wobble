#include <Arduino.h>
#include <Wire.h>
#include "joystick.h"
#include "encoder.h"
#include "display.h"
#include "communication.h"
/*
 *  PIN DEFINITIONS
 */
// Switches
const int LEFT_SW = 12;  // blue
const int LEFT_Y = 39;   // purple
const int LEFT_X = 36;   // gray
const int RIGHT_SW = 26; // blue
const int RIGHT_Y = 32;  // purple
const int RIGHT_X = 33;  // gray
// Encoder
const int ENC_SW = 13; // orange
const int ENC_CLK = 0; // green
const int ENC_DT = 4;  // yellow

// Objects
Joystick left_joystick(LEFT_SW, LEFT_X, LEFT_Y);
Joystick right_joystick(RIGHT_SW, RIGHT_X, RIGHT_Y);
Encoder dial(ENC_CLK, ENC_DT, ENC_SW);
Display display;
Communication socket;
uint8_t peer[6] = {0xEC, 0x64, 0xC9, 0x85, 0x70, 0x14};

int left_motor_power;
int right_motor_power;
Packet in_;
Packet out_;
Telemetry data_;
unsigned long last_recv_ms = 0;

void handleIncoming(const esp_now_recv_info_t *esp_now_info, const uint8_t *data, int len)
{
    memcpy(&in_, data, len);
    last_recv_ms = millis();
    if (in_.type == Message::Telemetry)
    {
        memcpy(&data_, in_.payload, in_.len);
        display.reset();
        display.printf("Pitch: %.2f\nL: %d  R: %d\n", data_.pitch, data_.left_enc, data_.right_enc);
        display.printf("Dial: %d", dial.read());
        display.printf("Power: %d, %d", left_motor_power, right_motor_power);
        display.update();
    }
}

void setup()
{
    Wire.begin();
    Serial.begin(115200);
    analogSetAttenuation(ADC_11db);
    socket.init();
    socket.addPeer(peer);
    socket.onReceive(handleIncoming);

    left_joystick.init(180, 190);
    right_joystick.init(220, 185);
    dial.init();
    display.init();
    display.reset();
    display.println(69);
    display.update();
}

void loop()
{
    if (millis() - last_recv_ms > 1000)
    {
        display.reset();
        display.printf("LOS\n");
        display.update();
    }
    // Serial.print("Lx: ");
    // Serial.print(-left_joystick.get_x());
    // Serial.print(" | Ly: ");
    // Serial.print(-left_joystick.get_y());
    // Serial.print(" | Rx: ");
    // Serial.print(-right_joystick.get_x());
    // Serial.print(" | Ry: ");
    // Serial.println(-right_joystick.get_y());
    // Read encoder value and set left motor speed
    int left_power = -left_joystick.get_x();
    int right_power = -right_joystick.get_x();
    left_motor_power = -left_joystick.get_x() * 255;
    right_motor_power = -right_joystick.get_x() * 255;
    out_.type = Message::Command;
    Command c{
        .left_wheel_power = left_motor_power,
        .right_wheel_power = right_motor_power,
    };
    out_.len = sizeof(c);
    memcpy(out_.payload, &c, out_.len);
    socket.send(peer, out_);
    delay(100);
}
