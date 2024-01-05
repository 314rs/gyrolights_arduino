#pragma once

#include <FastLED.h>
#include <projectConfig.h>
#include <effects.h>



extern CRGB leds[conf::NUM_STRIPS][conf::NUM_LEDS_PER_STRIP];


namespace conf {

    /**
     * @brief effects array
     * 
     * contains the effects mapped from the rotary switch.
     */
    const TaskFunction_t effects[NUM_PINS_ROTARY_SWITCH + 1] = {
        task_staticColor<*leds, conf::NUM_LEDS_TOTAL, CRGB::Black>, // first position, i.e. no pin connected.
    //    task_staticColor<*leds, conf::NUM_LEDS_TOTAL, CRGB::Red>, 
    //    task_case3<*leds, conf::NUM_LEDS_TOTAL>,
        task_gyroSimple<*leds, conf::NUM_LEDS_TOTAL>,
    //    task_staticColor<*leds, conf::NUM_LEDS_TOTAL, CRGB::Green>, 
    //    task_staticColor<*leds, conf::NUM_LEDS_TOTAL, CRGB::Blue>, 
    //    task_staticColor<*leds, conf::NUM_LEDS_TOTAL, CRGB::Yellow>,
    //    task_staticColor<*leds, conf::NUM_LEDS_TOTAL, CRGB::Magenta>,
        task_staticColor<*leds, conf::NUM_LEDS_TOTAL, CRGB::Cyan>,
        task_rainbow<*leds, conf::NUM_LEDS_TOTAL, 5>,
    //    task_gyroToHeatmap<*leds, conf::NUM_LEDS_TOTAL, heatmap_fire_>,
    };

}