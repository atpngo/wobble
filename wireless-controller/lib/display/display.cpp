#include "display.h"
#include <Arduino.h>
#include <Wire.h>
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C

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

void Display::printf(const char *fmt, ...)
{
    char buf[128]; // adjust size as needed
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    // send the formatted string to the OLED, supporting newlines
    const char *p = buf;
    while (*p)
    {
        // find end of line or end of string
        const char *e = strchr(p, '\n');
        if (!e)
            e = p + strlen(p);

        oled_.print(String(p).substring(0, e - p));
        oled_.println(); // move to next line
        if (*e == '\n')
            ++e;
        p = e;
    }
}