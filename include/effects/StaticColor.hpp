#include "Effect.hpp"
#include <FastLED.h>

class StaticColor : public Effect
{
private:
    CRGB* leds;
    int numleds;
    CRGB color;
public:
    StaticColor(CRGB* leds, int numleds, CRGB color);
    ~StaticColor();
    void init();
    void loop();
};
