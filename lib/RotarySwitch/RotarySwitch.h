#pragma once

#include <Arduino.h>
#include <driver/gpio.h>

template<int NUM_PINS> class RotarySwitch
{
private:
    gpio_num_t pins[NUM_PINS];
    bool levels[NUM_PINS];

    int lastState = -1;
    TickType_t lastStateTimestamp = 0;
    int stableState = -1;

    // callback ??

public:
    RotarySwitch();
    int getState();
    void update();

};

