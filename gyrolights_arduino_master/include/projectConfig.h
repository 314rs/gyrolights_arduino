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

const int NUM_LEDS_PER_STRIP = 5; ///< number of leds per individual strip
const int NUM_STRIPS = 1; ///< number of led strips
const uint8_t MAX_BRIGHTNESS = 80; ///< maximum led brightness value. from 0 (always off) to 255 (maximum possible brightness).

const int I2C_TIMEOUT_MS = 1000;

const float WMA_PARAM_ACCEL = 0.5; ///< parameter \f$\alpha\f$ in weighted moving average \f$(1-\alpha) * newValue + \alpha * oldValue \f$. lower value: more immediate response, higher value may reduce flicker. in range [0, 1]


const int UNIVERSE = 1; ///< First DMX Universe to listen for
const int UNIVERSE_COUNT = 1; ///< Total number of Universes to listen for, starting at UNIVERSE

const char BLE_SERVICE_UUID[] = "018c4715-a90b-7ff1-8c4c-aeed790b0a0a";
const char BLE_CHARACTERISTIC_UUID[] = "018c4715-f39b-7e59-8928-4a8eda02c6a5";


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
        0,   0,     0, 100, // blue
        255, 255,   0, 255, // yellow
};

DEFINE_GRADIENT_PALETTE(heatmap_tooff_) {
          0,   255,     200, 0, // blue
        255, 0,    0,    0, // yellow
};
