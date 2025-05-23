#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <freertos/FreeRTOS.h>
#include <freertos/portmacro.h>
#include "joystick.h"
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
Joystick left(LEFT_SW, LEFT_X, LEFT_Y);
Joystick right(RIGHT_SW, RIGHT_X, RIGHT_Y);

// Encoder variables
volatile int encoderValue = 0;
volatile uint8_t lastEncoded = 0;

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for SSD1306 display connected using I2C
#define OLED_RESET -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void IRAM_ATTR handleEncoder()
{
    portENTER_CRITICAL_ISR(&mux);
    uint8_t MSB = digitalRead(ENC_CLK);
    uint8_t LSB = digitalRead(ENC_DT);
    uint8_t encoded = (MSB << 1) | LSB;
    uint8_t sum = (lastEncoded << 2) | encoded;
    // clockwise steps
    if (sum == 0b1101 || sum == 0b0100 ||
        sum == 0b0010 || sum == 0b1011)
    {
        encoderValue++;
    }
    // counter-clockwise steps
    else if (sum == 0b1110 || sum == 0b0111 ||
             sum == 0b0001 || sum == 0b1000)
    {
        encoderValue--;
    }
    lastEncoded = encoded;
    portEXIT_CRITICAL_ISR(&mux);
}

void setup()
{
    Serial.begin(115200);
    analogSetAttenuation(ADC_11db);

    // Enable encoder inputs
    pinMode(ENC_DT, INPUT_PULLUP);
    pinMode(ENC_CLK, INPUT_PULLUP);
    pinMode(ENC_SW, INPUT_PULLUP);

    // read initial state so lastEncoded starts correctly
    lastEncoded = (digitalRead(ENC_CLK) << 1) | digitalRead(ENC_DT);

    // attach on both edges of both channels
    attachInterrupt(digitalPinToInterrupt(ENC_CLK), handleEncoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_DT), handleEncoder, CHANGE);

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
    display.println(encoderValue / 4);
    display.println(digitalRead(ENC_SW));

    display.display(); // send buffer to the screen
    delay(50);
}
