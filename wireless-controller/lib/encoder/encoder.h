#ifndef ENCODER_H
#define ENCODER_H
#include <freertos/FreeRTOS.h>
#include <freertos/portmacro.h>

class Encoder
{
public:
    Encoder(const int clock_pin, const int dt_pin, const int switch_pin);
    void init();
    int read();
    bool button_pressed();

private:
    const int clock_pin_;
    const int dt_pin_;
    const int switch_pin_;
    static Encoder *instance_;
    static void IRAM_ATTR isr();
    void handle_interrupt();
    portMUX_TYPE mux_ = portMUX_INITIALIZER_UNLOCKED;
    volatile int value_ = 0;
    volatile uint8_t last_encoded_ = 0;
};

#endif // ENCODER_H