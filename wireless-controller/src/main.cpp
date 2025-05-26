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
const int LEFT_Y = 14;   // white
const int LEFT_X = 27;   // gray
const int RIGHT_SW = 26; // white
const int RIGHT_Y = 25;  // green
const int RIGHT_X = 33;  // orange
// Encoder
const int ENC_SW = 13; // gray
const int ENC_CLK = 0; // yellow
const int ENC_DT = 4;  // green

// Objects
Joystick left_joystick(LEFT_SW, LEFT_X, LEFT_Y);
Joystick right_joystick(RIGHT_SW, RIGHT_X, RIGHT_Y);
Encoder dial(ENC_CLK, ENC_DT, ENC_SW);
Display display;
Communication socket;
uint8_t peer[6] = {0xEC, 0x64, 0xC9, 0x85, 0x70, 0x14};

volatile int count = 0;

Packet in_;
Packet out_;
Telemetry data_;
void handleIncoming(const esp_now_recv_info_t *esp_now_info, const uint8_t *data, int len)
{
    memcpy(&in_, data, len);
    Serial.print("Received bytes: ");
    if (in_.type == Message::Command)
    {
        // parse pkt.payload into your command enum/struct
    }
    else if (in_.type == Message::Telemetry)
    {
        count++;
        memcpy(&data_, in_.payload, sizeof(data_));
        display.reset();
        display.printf("Pitch: %.2f\nL: %d  R: %d", data_.pitch, data_.left_enc, data_.right_enc);
        display.update();
    }
}

void setup()
{
    Wire.begin();
    Serial.begin(115200);
    analogSetAttenuation(ADC_11db);
    Serial.println("init socket");
    socket.init();
    if (!socket.addPeer(peer))
    {
        Serial.println("Failed to add peer");
    }
    else
    {
        Serial.println("Added peer.");
    }
    socket.onReceive(handleIncoming);
    Serial.println("done socket");

    left_joystick.init(180, 190);
    right_joystick.init(220, 185);
    dial.init();
    display.init();
    display.reset();
    display.println(69);
    display.update();
    Serial.println("done");
}

void loop()
{
    // display.reset();
    // display.println(69);
    // display.update();
    // clear buffer and reset cursor to top‐left (or wherever you like)
    // Serial.print("Dial: ");
    // Serial.print(dial.read());
    // Serial.print(" | Pressed: ");
    // Serial.println(dial.button_pressed());
    // display.reset();
    // display.println(left_joystick.get_x());  // -180
    // display.println(left_joystick.get_y());  // -190
    // display.println(right_joystick.get_x()); // -220
    // display.println(right_joystick.get_y()); // -185

    // use print() instead of println() so it won’t advance down a line
    // display.println(dial.read());
    // display.println(dial.button_pressed());

    // display.update();
    // delay(50);
}
