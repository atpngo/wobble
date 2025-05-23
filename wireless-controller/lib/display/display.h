#ifndef DISPLAY_H
#define DISPLAY_H
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>
#include <string>

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
        oled_.println(v);
    }

private:
    Adafruit_SSD1306 oled_;
    const int SCREEN_WIDTH = 128;
    const int SCREEN_HEIGHT = 64;
    const int OLED_RESET = -1;
    const int SCREEN_ADDRESS = 0x3C;
};

#endif // DISPLAY_H