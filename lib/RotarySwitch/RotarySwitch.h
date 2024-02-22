#pragma once

///\todo remove Arduino dependency
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
    

    time_t debounceIntervall_uS;
    // callback ??

public:
    /**
     * @tparam NUM_PINS number of pins connected to the rotary switch
     * @param pins pointer to array of pins 
     */ 
    void init(const gpio_num_t* pins, time_t debounceIntervall_uS);
    int getState();
    void update();

};


template <unsigned int NUM_PINS>
void RotarySwitch<NUM_PINS>::init(const gpio_num_t* pins, time_t debounceIntervall_uS)
{
    this->debounceIntervall_uS = debounceIntervall_uS;
    memcpy(this->pins, pins, sizeof(gpio_num_t) * NUM_PINS);
    for (auto pin : this->pins) {
        pinMode(pin, INPUT_PULLUP);  ///@todo remove internal pullup or have this as an option
    } 
}


/**
 * @brief 
 * 
 * @tparam NUM_PINS 
 * @retval 0 if no pin is pulled low
 * @return number of connedted pin, 1-indexed
 */
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
        levels[i] = !digitalRead(pins[i]);
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
            /// @TODO: maybe add callback here?
        }
    }
}
