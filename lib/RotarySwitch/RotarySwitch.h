#pragma once

#include <Arduino.h>
#include <driver/gpio.h>

template<unsigned int NUM_PINS> class RotarySwitch
{
private:
    gpio_num_t pins[NUM_PINS];
    bool levels[NUM_PINS];

    int lastState = -1;
    int64_t lastStateTimestamp = 0;
    int stableState = -1;
    int64_t stableStateTimestamp = 0;
    

    static time_t debounceIntervall_uS;
    // callback ??

public:
    RotarySwitch();
    int getState();
    void update();

};


template <unsigned int NUM_PINS>
RotarySwitch<NUM_PINS>::RotarySwitch()
{
   /*  for (auto pin : pins) {
        pinMode(pin, INPUT);
    } */
}

template <unsigned int NUM_PINS>
int RotarySwitch<NUM_PINS>::getState()
{   
    this->update();
    return this->stableState;
}

template <unsigned int NUM_PINS>
void RotarySwitch<NUM_PINS>::update()
{
    unsigned int sum = 0;
    int measuredState = 0;
    for (int i = 0; i < NUM_PINS; i++) {
        levels[i] = digitalRead(pins[i]);
        if (!sum)
            measuredState++;
        sum += levels[i];
    }
    if (sum <= 1) {
        measuredState *= sum;
        uint64_t currentTime = esp_timer_get_time();
        if (lastState != measuredState) {
            lastState = measuredState;
            lastStateTimestamp = currentTime;
        } else if (currentTime - lastStateTimestamp > debounceIntervall_uS) {
            stableState = measuredState;
            stableStateTimestamp = measuredState;
            // state updated!
        }
    }
}
