/**
 * @file projectConfig.h
 * @author 314rs (lars.borntraeger@gmx.de)
 * @brief project specific configuration parameters
 * @version 0.1
 * @date 2023-09-13
 * 
 */

#pragma once


#include <hal/gpio_types.h>
#include <esp_err.h>
#include <FastLED.h>
#include <driver/ledc.h>


#include "effects/Effect.hpp"


/**
 * All important configuration parameters are in the namespace `conf`
 */
namespace conf
{

/* -------------------------------------------------------------------------- */
/*                          configuration parameters                          */
/* -------------------------------------------------------------------------- */

const char WIFI_AP_SSID_PREFIX[] = "ESP32_"; ///< first half of the access point ssid. second half are the last 6 characters of the mac address in hex.
const char WIFI_AP_PW[] = "password"; ///< password of the configuration access point

const int NUM_LEDS_PER_STRIP = 21; ///< number of leds per individual strip
const int NUM_STRIPS = 1; ///< number of led strips
const uint8_t MAX_BRIGHTNESS = 80; ///< maximum led brightness value. from 0 (always off) to 255 (maximum possible brightness).

const int I2C_TIMEOUT_MS = 1000;

const float WMA_PARAM_ACCEL = 0.5; ///< parameter \f$\alpha\f$ in weighted moving average \f$(1-\alpha) * newValue + \alpha * oldValue \f$. lower value: more immediate response, higher value may reduce flicker. in range [0, 1]


const int UNIVERSE = 1; ///< First DMX Universe to listen for
const int UNIVERSE_COUNT = 1; ///< Total number of Universes to listen for, starting at UNIVERSE


const char BLE_MASTER_NAME[] = "ESP_32"; ///< this has to be the same name for a pair of lights


/* -------------------------------------------------------------------------- */
/*                                    Pins                                    */
/* -------------------------------------------------------------------------- */

const gpio_num_t LED_PIN = GPIO_NUM_2;


const gpio_num_t PINS_ROTARY_SWITCH[] = {
    GPIO_NUM_36,
    GPIO_NUM_39,
    GPIO_NUM_18,
    GPIO_NUM_17,
    GPIO_NUM_16,
    GPIO_NUM_15,
    GPIO_NUM_14,
    GPIO_NUM_13,
    GPIO_NUM_34
};
///< all rotary switch pins are pulled up in hardware

const gpio_num_t PIN_LED_CLOCK = GPIO_NUM_19;
const gpio_num_t PIN_LED_DATA_1 = GPIO_NUM_23;
const gpio_num_t PIN_LED_DATA_2 = GPIO_NUM_25;
const gpio_num_t PIN_LED_DATA_3 = GPIO_NUM_26;
const gpio_num_t PIN_LED_DATA_4 = GPIO_NUM_27;
const gpio_num_t PIN_LED_DATA_5 = GPIO_NUM_32;
const gpio_num_t PIN_LED_DATA_6 = GPIO_NUM_33;

const gpio_num_t PIN_RF_SWITCH = GPIO_NUM_35; ///< Input Pullup in Hardware

const gpio_num_t PIN_MPU6050_SCL = GPIO_NUM_22;
const gpio_num_t PIN_MPU6050_SDA = GPIO_NUM_21;


/* -------------------------------------------------------------------------- */
/*                                Do not change                               */
/* -------------------------------------------------------------------------- */

const int NUM_PINS_ROTARY_SWITCH = (sizeof(PINS_ROTARY_SWITCH)/sizeof(*PINS_ROTARY_SWITCH));

const int NUM_LEDS_TOTAL = (NUM_LEDS_PER_STRIP * NUM_STRIPS);

const int I2C_GYRO_ADDR = 0x68;

const int GYRO_ACCEL_SENSITIVITY = 2; ///< sensitivity of the accelerometer range: 0..3

const int G_VAL = (2 << (15 - 2 - GYRO_ACCEL_SENSITIVITY));  


// uuids from https://www.uuidgenerator.net/ 
const char BLE_SERVICE_UUID[] = "018c4715-a90b-7ff1-8c4c-aeed790b0a0a";
const char BLE_CHARACTERISTIC_UUID[] = "018c4715-f39b-7e59-8928-4a8eda02c6a5";

} // namespace conf


/**
 * @brief read values from gyro sensor
 * 
 * @param[in] rxvalues create int16_t rxvalues[7] before and pass pointer to it. will be filled with values for accelx, accely accelz, temp, gyrox, gyroy, gyroz
 */
void readGyro(int16_t *rxvalues) {
    static const uint8_t startregister = 0x3B;
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
 * @brief let the onboard led sine-fade
 */
void ledFun(void*) {
    esp_log_write(ESP_LOG_DEBUG, __FUNCTION__, "%s started", __FUNCTION__);
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_HIGH_SPEED_MODE,
        .duty_resolution  = LEDC_TIMER_15_BIT,
        .timer_num        = LEDC_TIMER_0,
        .freq_hz          = 1000,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));
    ledc_channel_config_t ledc_channel = {
        .gpio_num       = GPIO_NUM_2,
        .speed_mode     = LEDC_HIGH_SPEED_MODE,
        .channel        = LEDC_CHANNEL_0,
        .intr_type      = LEDC_INTR_DISABLE,
        .timer_sel      = LEDC_TIMER_0,
        .duty           = 0,
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
    int16_t val = 0;
    while (true)
    {
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, abs(val)));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0));
        val+=100;
        vTaskDelay(1);
    }
}

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


static CRGB leds[conf::NUM_STRIPS][conf::NUM_LEDS_PER_STRIP];

static int8_t rotaryswitch = -1;




/**
 * @brief handle local tasks
 * 
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

    


    while (1) {
        ESP_LOG_LEVEL(ESP_LOG_VERBOSE, __FUNCTION__, "high watermark: %d, rotaryswitch: %d", uxTaskGetStackHighWaterMark(NULL), rotaryswitch);
        FastLED.show(conf::MAX_BRIGHTNESS);
        /**
         * @todo 
         * line @lineinfo: replace  switch with array of function pointers and
         * put array in projectConfig.h
         */
        switch (rotaryswitch) {
        case 0:
            readGyro(rxvalues);
            r = ((1-alpha) * r ) + (alpha * (abs(rxvalues[0]) >> 7));
            g = ((1-alpha) * g ) + (alpha * (abs(rxvalues[1]) >> 7));
            b = ((1-alpha) * b ) + (alpha * (abs(rxvalues[2]) >> 7));
            fill_solid(*leds, conf::NUM_LEDS_TOTAL, CRGB(r, g, b));
            //FastLED.show();
            break;
        case 1:
            /* ---------------------------------- jerk ---------------------------------- */
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
            fill_solid(*leds, conf::NUM_LEDS_TOTAL, CRGB::White);
            // FastLED.show(mapfb(jerk, jerk_min, jerk_max, 0, 255));
            //FastLED.show(255 * (jerk > 100.0));
            break;
        case 2:
            fill_solid(*leds, conf::NUM_LEDS_TOTAL, CRGB::Green);
            readGyro(rxvalues);
            ESP_LOG_LEVEL(ESP_LOG_INFO, "readGyro", "accelx: %7d, accely: %7d, accelz: %7d, temp: %7d, gyrox: %7d, gyroy: %7d, gyroz: %7d\r", (rxvalues[0]), (rxvalues[1]), (rxvalues[2]), (rxvalues[3]), (rxvalues[4]), (rxvalues[5]), (rxvalues[6]));

            //FastLED.show(conf::MAX_BRIGHTNESS);
            //vTaskSuspend(NULL);
            break;
        case 3:
            directednormaccel = directedAccelNorm();
            if (directednormaccel < -10000) {
                fill_solid(*leds, conf::NUM_LEDS_TOTAL, CRGB::Green);
            } else {
                fill_solid(*leds, conf::NUM_LEDS_TOTAL, CRGB::Black);
            }
            ESP_LOG_LEVEL(ESP_LOG_INFO, "accel", "acceleration norm: %f\r", directednormaccel);
            //FastLED.show(conf::MAX_BRIGHTNESS);
            //vTaskSuspend(NULL);
            break;
        case 4:
            fill_solid(*leds, conf::NUM_LEDS_TOTAL, CRGB::Lime);
            //FastLED.show(conf::MAX_BRIGHTNESS);
            //vTaskSuspend(NULL);
            break;
        case 5:
            readGyro(rxvalues);
            
            norm_reading = sqrtf(pow(rxvalues[0], 2) + pow(rxvalues[1], 2) + pow(rxvalues[2], 2)) - conf::G_VAL;
            if (norm_accel_max < norm_accel) 
                norm_accel_max = norm_accel;
            if (norm_reading < norm_accel + 200) {
                norm_accel = (norm_reading * alpha) +  (norm_accel * (1.0 - alpha));
            } else {
                norm_accel = norm_reading;
            }
            fill_solid(*leds, conf::NUM_LEDS_TOTAL, ColorFromPalette(tooff, 0xff * (norm_accel/norm_accel_max)));
            //FastLED.show();
            break;
        case 6:
            readGyro(rxvalues);
            
            norm_reading = sqrtf(pow(rxvalues[0], 2) + pow(rxvalues[1], 2) + pow(rxvalues[2], 2)) - conf::G_VAL;
            if (norm_accel_max < norm_accel) 
                norm_accel_max = norm_accel;
            if (norm_reading < norm_accel + 200) {
                norm_accel = (norm_reading * alpha) +  (norm_accel * (1.0 - alpha));
            } else {
                norm_accel = norm_reading;
            }
            fill_solid(*leds, conf::NUM_LEDS_TOTAL, ColorFromPalette(fire, 0xff * (norm_accel/norm_accel_max)));
            //FastLED.show();
            break;
        case 7:
            readGyro(rxvalues);
            ESP_LOG_LEVEL(ESP_LOG_INFO, "readGyro", "accelx: %7d, accely: %7d, accelz: %7d, temp: %7d, gyrox: %7d, gyroy: %7d, gyroz: %7d\r", (rxvalues[0]), (rxvalues[1]), (rxvalues[2]), (rxvalues[3]), (rxvalues[4]), (rxvalues[5]), (rxvalues[6]));
            directednormaccel = directedAccelNorm();
            if (directednormaccel < -10000) {
                brightness = conf::MAX_BRIGHTNESS;
                fill_solid(*leds, conf::NUM_LEDS_TOTAL, CRGB::Cyan);
            } else {
                brightness = brightness * 0.95;
            }
            ESP_LOG_LEVEL(ESP_LOG_INFO, "accel", "acceleration norm: %f\r", directednormaccel);
            //FastLED.show();
            //vTaskSuspend(NULL);
            break;
        
        case 8:
            readGyro(rxvalues);
            
            norm_reading = sqrtf(pow(rxvalues[0], 2) + pow(rxvalues[1], 2) + pow(rxvalues[2], 2)) - conf::G_VAL;
            if (norm_accel_max < norm_accel) 
                norm_accel_max = norm_accel;
            if (norm_reading < norm_accel + 200) {
                norm_accel = (norm_reading * alpha) +  (norm_accel * (1.0 - alpha));
            } else {
                norm_accel = norm_reading;
            }
            fill_solid(*leds, conf::NUM_LEDS_TOTAL, ColorFromPalette(test, 0xff * (norm_accel/norm_accel_max)));
            
            break;

        
        default:
            fill_solid(*leds, conf::NUM_LEDS_TOTAL, CRGB::Black);
            //FastLED.show();
            //vTaskSuspend(NULL);
            break;
        }
        vTaskDelay(5);
    }
}