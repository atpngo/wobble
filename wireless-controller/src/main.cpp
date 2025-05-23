#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <freertos/FreeRTOS.h>
#include <freertos/portmacro.h>
#include "joystick.h"
#include "encoder.h"
// define your mux (must be unlocked at start)
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

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

// Encoder variables
volatile int encoderValue = 0;
volatile uint8_t lastEncoded = 0;

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for SSD1306 display connected using I2C
#define OLED_RESET -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void setup()
{
    Serial.begin(115200);
    analogSetAttenuation(ADC_11db);

    left_joystick.init();
    right_joystick.init();
    dial.init();

    // initialize the OLED object
    if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS))
    {
        Serial.println(F("SSD1306 allocation failed"));
        for (;;)
            ;
    }

    // Clear the buffer.
    display.clearDisplay();

    // // Display Text
    display.stopscroll();
    display.setTextSize(1);      // Set text size
    display.setTextColor(WHITE); // Set text color
    display.setCursor(0, 28);    // Set cursor position
}

void loop()
{
    // clear buffer and reset cursor to top‐left (or wherever you like)
    display.clearDisplay();
    display.setCursor(0, 28);

    // use print() instead of println() so it won’t advance down a line
    display.println(dial.read());
    display.println(dial.button_pressed());

    display.display(); // send buffer to the screen
    delay(50);
}
