#include "display.h"
#include <Wire.h>

Display::Display() : oled_(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET)
{
}

void Display::init()
{
    if (!oled_.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS))
    {
        Serial.println(F("SSD1306 allocation failed"));
        for (;;)
            ;
    }

    oled_.stopscroll();
    oled_.setTextSize(1);
    oled_.setTextColor(WHITE);
    reset();
}

void Display::reset()
{
    oled_.clearDisplay();
    oled_.setCursor(0, 28);
}

void Display::update()
{
    oled_.display();
}