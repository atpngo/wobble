#include "encoder.h"
#include <Arduino.h>

Encoder::Encoder(int clock_pin,
                 int dt_pin,
                 int switch_pin) : clock_pin_(clock_pin),
                                   dt_pin_(dt_pin),
                                   switch_pin_(switch_pin)
{
}

Encoder::Encoder(int clock_pin,
                 int dt_pin) : clock_pin_(clock_pin),
                               dt_pin_(dt_pin),
                               switch_pin_(-1)
{
}

void Encoder::init()
{
    pinMode(clock_pin_, INPUT_PULLUP);
    pinMode(dt_pin_, INPUT_PULLUP);
    if (switch_pin_ != -1)
    {
        pinMode(switch_pin_, INPUT_PULLUP);
    }
    last_encoded_ = (digitalRead(clock_pin_) << 1) | digitalRead(dt_pin_);
    attachInterruptArg(digitalPinToInterrupt(clock_pin_), Encoder::isr_clk, this, CHANGE);
    attachInterruptArg(digitalPinToInterrupt(dt_pin_), Encoder::isr_dt, this, CHANGE);
}

int Encoder::read()
{
    int v;
    portENTER_CRITICAL(&mux_);
    v = value_;
    portEXIT_CRITICAL(&mux_);
    return v / 4;
}

bool Encoder::button_pressed()
{
    if (switch_pin_ == -1)
        return false;

    return digitalRead(switch_pin_) == LOW;
}

void IRAM_ATTR Encoder::isr_clk(void *arg)
{
    static_cast<Encoder *>(arg)->handle_interrupt();
}

void IRAM_ATTR Encoder::isr_dt(void *arg)
{
    static_cast<Encoder *>(arg)->handle_interrupt();
}

void IRAM_ATTR Encoder::handle_interrupt()
{
    portENTER_CRITICAL_ISR(&mux_);
    uint8_t MSB = digitalRead(clock_pin_);
    uint8_t LSB = digitalRead(dt_pin_);
    uint8_t encoded = (MSB << 1) | LSB;
    uint8_t sum = (last_encoded_ << 2) | encoded;
    // clockwise steps
    if (sum == 0b1101 || sum == 0b0100 ||
        sum == 0b0010 || sum == 0b1011)
    {
        value_++;
    }
    // counter-clockwise steps
    else if (sum == 0b1110 || sum == 0b0111 ||
             sum == 0b0001 || sum == 0b1000)
    {
        value_--;
    }
    last_encoded_ = encoded;
    portEXIT_CRITICAL_ISR(&mux_);
}