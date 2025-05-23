#ifndef DISPLAY_H
#define DISPLAY_H
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>
#include <string>
#include "util.h"

class Display
{
public:
    Display();
    void init();
    void reset();
    void update();

    template <typename T>
    void println(T v)
    {
        oled_.println(approximate_to_zero(v, 1e-2));
    }

private:
    Adafruit_SSD1306 oled_;
};

#endif // DISPLAY_H