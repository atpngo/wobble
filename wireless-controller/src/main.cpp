#include <Arduino.h>
#include "joystick.h"
#include "encoder.h"
#include "display.h"

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

void setup()
{
    Serial.begin(115200);
    analogSetAttenuation(ADC_11db);

    left_joystick.init();
    right_joystick.init();
    dial.init();
    display.init();
}

void loop()
{
    // clear buffer and reset cursor to top‐left (or wherever you like)
    display.reset();

    // use print() instead of println() so it won’t advance down a line
    display.println(dial.read());
    display.println(dial.button_pressed());

    display.update();
    delay(50);
}
