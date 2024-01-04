#pragma once

#include <Arduino.h>
#include <FastLED.h>
#include <esp_log.h>
#include <lwip/def.h>
#include <driver/i2c.h>

#include <projectConfig.h>

extern TaskHandle_t task_local;

extern TProgmemRGBGradientPalette_bytes heatmap_fire_;
extern TProgmemRGBGradientPalette_bytes heatmap_test_;
extern TProgmemRGBGradientPalette_bytes heatmap_tooff_;

CRGBPalette16 fire = heatmap_fire_;
CRGBPalette16 test = heatmap_test_;
CRGBPalette16 tooff = heatmap_tooff_;


/**
 * @brief read values from gyro sensor
 * 
 * @param[in] rxvalues create int16_t rxvalues[7] before and pass pointer to it. will be filled with values for accelx, accely accelz, temp, gyrox, gyroy, gyroz
 */
void readGyro(int16_t *rxvalues) {
    const uint8_t startregister = 0x3B;
    uint8_t rx_data[14];
    ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_master_write_read_device(I2C_NUM_0, conf::I2C_GYRO_ADDR, &startregister, sizeof(startregister), rx_data, sizeof(rx_data), conf::I2C_TIMEOUT_MS/portTICK_RATE_MS));
    //ESP_LOG_BUFFER_HEX_LEVEL("i2c_gyro_rx", rx_data, sizeof(rx_data), ESP_LOG_VERBOSE);
    for (int i = 0; i < 7; i++,rxvalues++) {
        *rxvalues = htons(*((int16_t*) rx_data+i));
    }
    ESP_LOG_LEVEL(ESP_LOG_VERBOSE, "readGyro", "accelx: %7d, accely: %7d, accelz: %7d, temp: %7d, gyrox: %7d, gyroy: %7d, gyroz: %7d\r", (rxvalues[0]), (rxvalues[1]), (rxvalues[2]), (rxvalues[3]), (rxvalues[4]), (rxvalues[5]), (rxvalues[6]));
    //rxvalues = reinterpret_cast<int16_t*>(rx_data);
}

/**
 * @return Norm of the acceleration vector if acceleration in x direction is positive, negative norm oterwise.
 */
float directedAccelNorm() {
    int16_t rxvalues[7];
    readGyro(rxvalues);
    const int sign = rxvalues[0] < 0 ? (-1) : 1;
    return sign * sqrtf(pow(rxvalues[0], 2) + pow(rxvalues[1], 2) + pow(rxvalues[2], 2));
}

/**
 * @brief scales a float variable in [in_min, in_max] linearly to [out_min, out_max]
 * 
 * @param in input value
 * @param in_min input lower bound
 * @param in_max input upper bound
 * @param out_min output lower bound
 * @param out_max output upper bound
 * @retval out_min if `in` <= `in_min`
 * @retval out_max if `in` >= `in_max`
 * @return mapped value
 */
float mapfb(float in, float in_min, float in_max, float out_min, float out_max) {
    if (in <= in_min)
        return out_min;
    if (in_min >= in_max)
        return out_max;
    return (in - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}



/**
 * @brief task to apply FastLEDs fill_rainbow to `targetArray` with changing start hue.
 * 
 * @tparam targetArray a pointer to the LED array to fill
 * @tparam numToFill the number of LEDs to fill in the array
 * @tparam PERIOD_LENGTH in seconds; approx. time, after which the colors repeat.
 */
template<CRGB* targetArray, uint numToFill, uint PERIOD_LENGTH>
void task_rainbow(void*) {
    ESP_LOG_LEVEL(ESP_LOG_DEBUG, __func__, "new mode");
    int hue = 0;
    const int mydelay = PERIOD_LENGTH * 4;  // approx. PERIOD_LENGTH * 1000 / 256. (nach 256 schleifen soll wieder die start-hue sein. das soll Period length dauern)
    while(true) {
        fill_rainbow(targetArray, numToFill, hue);
        hue++;
        FastLED.show();
        vTaskDelay(pdMS_TO_TICKS(mydelay));
    }
}

/**
 * @brief task to fill `targetArray` with a color, then delete task
 * 
 * @tparam targetArray a pointer to the LED array to fill
 * @tparam numToFill the number of LEDs to fill in the array
 * @tparam colorcode the color. Can be of type CRGB (implicit cast)
 */
template<CRGB* targetArray, uint numToFill, uint32_t colorcode>
void task_staticColor(void*) {
    ESP_LOG_LEVEL(ESP_LOG_DEBUG, __func__, "new mode");
    fill_solid(targetArray, numToFill, CRGB(colorcode));
    FastLED.show();
    task_local = NULL;
    vTaskDelete(NULL);
}

/**
 * @brief task maps x,y,z-readings form gyro to r,g,b of `targetArray`
 * 
 * @tparam targetArray a pointer to the LED array to fill
 * @tparam numToFill the number of LEDs to fill in the array
 */
template<CRGB* targetArray, uint numToFill>
void task_gyroSimple(void*) {
    int16_t rxvalues[7];
    static uint8_t r, g, b;
    const float alpha = 1.0/16.0;
    while (true) {
        readGyro(rxvalues);
        r = ((1-alpha) * r ) + (alpha * (abs(rxvalues[0]) >> 7));
        g = ((1-alpha) * g ) + (alpha * (abs(rxvalues[1]) >> 7));
        b = ((1-alpha) * b ) + (alpha * (abs(rxvalues[2]) >> 7));
        fill_solid(targetArray, numToFill, CRGB(r, g, b));
        FastLED.show();
        vTaskDelay(34);
    }
}


/**
 * @brief task maps acceleration to heatmap.
 * 
 * @tparam targetArray a pointer to the LED array to fill
 * @tparam numToFill the number of LEDs to fill in the array
 */
template<CRGB* targetArray, uint numToFill, TProgmemRGBGradientPalette_bytes heatmap>
void task_gyroToHeatmap(void*) {
    while (true) {
        vTaskDelay(34);
    }
}


/**
 * @brief legacy stuff
 */
void localtask(void*) {
    ESP_LOG_LEVEL(ESP_LOG_DEBUG, __FUNCTION__, "%s started", __FUNCTION__);
    // case 0
    int16_t rxvalues[7];
    static uint8_t r, g, b;
    const float alpha = 1.0/16.0;

    // case 6
    unsigned long time_now, time_prev;
    time_now = time_prev = millis();
    float norm_accel_max = 0, norm_reading = 0, norm_prev = 0;
    static float norm_accel = 0;
    CRGBPalette16 fire = heatmap_fire_;
    CRGBPalette16 test = heatmap_test_;
    CRGBPalette16 tooff = heatmap_tooff_;
    float jerk = 0;
    static float jerk_prev = 0, jerk_max = 0, jerk_min = 0;
    float directednormaccel = 0;
    uint8_t brightness = 0;
}

template<CRGB* targetArray, uint numToFill>
void task_case1(void*) {
    /* ---------------------------------- jerk ---------------------------------- */
    unsigned long time_now, time_prev;
    time_now = time_prev = millis();
    float norm_accel_max = 0, norm_reading = 0, norm_prev = 0;
    float jerk = 0;
    static float jerk_prev = 0, jerk_max = 0, jerk_min = 0;
    while (true) {
        time_now = millis();
        norm_reading = directedAccelNorm();
        jerk = (norm_reading - norm_prev)/(time_now-time_prev);
        time_prev = time_now;
        norm_prev = norm_reading;
        // bounds
        jerk_max = jerk_max * 0.99;
        jerk_min = jerk_min * 0.99;
        if (jerk > jerk_max) jerk_max = jerk;
        else if (jerk < jerk_min) jerk_min = jerk;
        ESP_LOG_LEVEL(ESP_LOG_DEBUG, __FUNCTION__, "jerk: %f", jerk);
        fill_solid(targetArray, numToFill, CRGB::White);
        FastLED.show(mapfb(jerk, jerk_min, jerk_max, 0, 255));
        //FastLED.show(255 * (jerk > 100.0));
        vTaskDelay(34);
    }
}

template<CRGB* targetArray, uint numToFill>
void task_case3(void*) {
    float directednormaccel = 0;
    while (true) {
        directednormaccel = directedAccelNorm();
        if (directednormaccel < -10000) {
            fill_solid(targetArray, numToFill, CRGB::Green);
        } else {
            fill_solid(targetArray, numToFill, CRGB::Black);
        }
        ESP_LOG_LEVEL(ESP_LOG_INFO, "accel", "acceleration norm: %f\r", directednormaccel);
        FastLED.show();
        vTaskDelay(34);
    }
}

template<CRGB* targetArray, uint numToFill>
void task_case5(void*) {
    int16_t rxvalues[7];
    const float alpha = 1.0/16.0;
    float norm_accel_max = 0, norm_reading = 0, norm_prev = 0;
    static float norm_accel = 0;
    while (true) {
        readGyro(rxvalues);
            
        norm_reading = sqrtf(pow(rxvalues[0], 2) + pow(rxvalues[1], 2) + pow(rxvalues[2], 2)) - conf::G_VAL;
        if (norm_accel_max < norm_accel) 
            norm_accel_max = norm_accel;
        if (norm_reading < norm_accel + 200) {
            norm_accel = (norm_reading * alpha) +  (norm_accel * (1.0 - alpha));
        } else {
            norm_accel = norm_reading;
        }
        fill_solid(targetArray, numToFill, ColorFromPalette(tooff, 0xff * (norm_accel/norm_accel_max)));
        //FastLED.show();
        vTaskDelay(34);
    }
}

template<CRGB* targetArray, uint numToFill>
void task_case6(void*) {
    int16_t rxvalues[7];
    const float alpha = 1.0/16.0;
    float norm_accel_max = 0, norm_reading = 0, norm_prev = 0;
    static float norm_accel = 0;
    while (true) {
        readGyro(rxvalues);
            
        norm_reading = sqrtf(pow(rxvalues[0], 2) + pow(rxvalues[1], 2) + pow(rxvalues[2], 2)) - conf::G_VAL;
        if (norm_accel_max < norm_accel) 
            norm_accel_max = norm_accel;
        if (norm_reading < norm_accel + 200) {
            norm_accel = (norm_reading * alpha) +  (norm_accel * (1.0 - alpha));
        } else {
            norm_accel = norm_reading;
        }
        fill_solid(targetArray, numToFill, ColorFromPalette(fire, 0xff * (norm_accel/norm_accel_max)));
        //FastLED.show();
        vTaskDelay(34);
    }
}

template<CRGB* targetArray, uint numToFill>
void task_case7(void*) {
    int16_t rxvalues[7];
    float directednormaccel = 0;
    uint8_t brightness = 0;
    while (true) {
        readGyro(rxvalues);
        ESP_LOG_LEVEL(ESP_LOG_INFO, "readGyro", "accelx: %7d, accely: %7d, accelz: %7d, temp: %7d, gyrox: %7d, gyroy: %7d, gyroz: %7d\r", (rxvalues[0]), (rxvalues[1]), (rxvalues[2]), (rxvalues[3]), (rxvalues[4]), (rxvalues[5]), (rxvalues[6]));
        directednormaccel = directedAccelNorm();
        if (directednormaccel < -10000) {
            brightness = conf::MAX_BRIGHTNESS;
            fill_solid(targetArray, numToFill, CRGB::Cyan);
        } else {
            brightness = brightness * 0.95;
        }
        ESP_LOG_LEVEL(ESP_LOG_INFO, "accel", "acceleration norm: %f\r", directednormaccel);
        //FastLED.show();
        //vTaskSuspend(NULL);
        vTaskDelay(34);
    }
}

template<CRGB* targetArray, uint numToFill>
void task_case8(void*) {
    int16_t rxvalues[7];
    float norm_accel_max = 0, norm_reading = 0, norm_prev = 0;
    static float norm_accel = 0;
    while (true) {
        readGyro(rxvalues);
            
        norm_reading = sqrtf(pow(rxvalues[0], 2) + pow(rxvalues[1], 2) + pow(rxvalues[2], 2)) - conf::G_VAL;
        if (norm_accel_max < norm_accel) 
            norm_accel_max = norm_accel;
        if (norm_reading < norm_accel + 200) {
            norm_accel = (norm_reading * alpha) +  (norm_accel * (1.0 - alpha));
        } else {
            norm_accel = norm_reading;
        }
        fill_solid(targetArray, numToFill, ColorFromPalette(test, 0xff * (norm_accel/norm_accel_max)));
        
        vTaskDelay(34);
    }
}