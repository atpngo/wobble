#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

const int POT_SIGNAL_PIN = 36;
const int LEFT_SW = 12;  // blue
const int LEFT_Y = 14;   // white
const int LEFT_X = 27;   // gray
const int RIGHT_SW = 26; // white
const int RIGHT_Y = 25;  // green
const int RIGHT_X = 33;  // orange

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for SSD1306 display connected using I2C
#define OLED_RESET -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

float floatMap(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void setup()
{
    // put your setup code here, to run once:
    Serial.begin(115200);
    pinMode(LEFT_SW, INPUT_PULLUP);
    pinMode(RIGHT_SW, INPUT_PULLUP);
    analogSetAttenuation(ADC_11db);

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
    // put your main code here, to run repeatedly:
    int analogValue = analogRead(36);
    // Rescale to potentiometer's voltage (from 0V to 3.3V):
    float voltage = floatMap(analogValue, 0, 4095, 0, 3.3);
    // Serial.print("Analog: ");
    // Serial.print(analogValue);
    // Serial.print(", Voltage: ");
    // Serial.println(voltage);

    // Serial.print("[LEFT] X: ");
    // Serial.print(analogRead(LEFT_X)); // print the value of VRX
    // Serial.print("|Y: ");
    // Serial.print(analogRead(LEFT_Y)); // print the value of VRX
    // Serial.print("|Z: ");
    // Serial.println(digitalRead(LEFT_SW)); // print the value of SW

    // Serial.print("[RIGHT] X: ");
    // Serial.print(analogRead(RIGHT_X)); // print the value of VRX
    // Serial.print("|Y: ");
    // Serial.print(analogRead(RIGHT_Y)); // print the value of VRX
    // Serial.print("|Z: ");
    // Serial.println(digitalRead(RIGHT_SW)); // print the value of SW

    // clear buffer and reset cursor to top‐left (or wherever you like)
    display.clearDisplay();
    display.setCursor(0, 28);

    // use print() instead of println() so it won’t advance down a line
    display.print(voltage, 2); // ‘2’ is number of decimals

    display.display(); // send buffer to the screen
    delay(50);
}
