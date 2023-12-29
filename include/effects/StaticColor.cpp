#include "StaticColor.hpp"

StaticColor::StaticColor(CRGB *leds, int numleds, CRGB color) :
leds(leds), numleds(numleds), color(color)
{
}

void StaticColor::init()
{
    fill_solid(leds, numleds, color);
    FastLED.show(maxBrightness);
}

void StaticColor::loop() {
}