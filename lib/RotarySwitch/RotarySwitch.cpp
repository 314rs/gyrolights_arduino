#include <RotarySwitch.h>

template <int NUM_PINS>
RotarySwitch<NUM_PINS>::RotarySwitch()
{
    for (auto pin : pins) {
        pinMode(pin, INPUT);
    }
}

template <int NUM_PINS>
inline int RotarySwitch<NUM_PINS>::getState()
{   
    this->update();
    return this->stableState;
}

template <int NUM_PINS>
inline void RotarySwitch<NUM_PINS>::update()
{
    int sum = 0;
    for (int i = 0; i < NUM_PINS; i++) {
        levels[i] = digitalRead(pins[i]);
        sum += levels[i];

    }
}
