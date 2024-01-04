#pragma once

#include <FastLED.h>
#include <projectConfig.h>
#include <effects.h>



extern CRGB leds[conf::NUM_STRIPS][conf::NUM_LEDS_PER_STRIP];

DEFINE_GRADIENT_PALETTE(heatmap_fire_) {
           0,   0,   0,   0, // black
         128, 255,   0,   0,   //red
         224, 255, 255,   0,   //bright yellow
         255, 255, 255, 255, // white
    };

DEFINE_GRADIENT_PALETTE(heatmap_test_) {
           0,   0, 255,   0, // blue
         255, 255,   0,   0, // yellow
};

DEFINE_GRADIENT_PALETTE(heatmap_tooff_) {
           0,   0, 255,   0, // blue
         255,   0,   0,   0, // yellow
};

namespace conf {

    /**
     * @brief effects array
     * 
     * contains the effects mapped from the rotary switch.
     */
    const TaskFunction_t effects[NUM_PINS_ROTARY_SWITCH + 1] = {
        task_staticColor<*leds, conf::NUM_LEDS_TOTAL, CRGB::Black>, // first position, i.e. no pin connected.
    //    task_staticColor<*leds, conf::NUM_LEDS_TOTAL, CRGB::Red>, // second position = pin 1
        task_case3<*leds, conf::NUM_LEDS_TOTAL>,
        task_gyroSimple<*leds, conf::NUM_LEDS_TOTAL>,
    //    task_staticColor<*leds, conf::NUM_LEDS_TOTAL, CRGB::Green>, // pin 2
    //    task_staticColor<*leds, conf::NUM_LEDS_TOTAL, CRGB::Blue>, // ...
    //    task_staticColor<*leds, conf::NUM_LEDS_TOTAL, CRGB::Yellow>,
    //    task_staticColor<*leds, conf::NUM_LEDS_TOTAL, CRGB::Magenta>,
    //    task_staticColor<*leds, conf::NUM_LEDS_TOTAL, CRGB::Cyan>,
    //    task_rainbow<*leds, conf::NUM_LEDS_TOTAL, 5>,
        task_gyroToHeatmap<*leds, conf::NUM_LEDS_TOTAL, heatmap_fire_>,
    };

}