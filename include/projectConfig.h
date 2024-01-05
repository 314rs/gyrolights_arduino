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






/**
 * All important configuration parameters are in the namespace `conf`
 */
namespace conf
{

/* -------------------------------------------------------------------------- */
/*                          configuration parameters                          */
/* -------------------------------------------------------------------------- */

const char WIFI_AP_SSID_PREFIX[] = "ESP32_"; ///< first part of the access point ssid. second part are the last 6 characters of the mac address in hex.
const char WIFI_AP_PW[] = "password1234"; ///< password of the configuration access point

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
//    GPIO_NUM_36,
//    GPIO_NUM_39,
    GPIO_NUM_18,
    GPIO_NUM_17,
    GPIO_NUM_16,
//    GPIO_NUM_15,      // JTAG PIN
//    GPIO_NUM_14,      // JTAG PIN
//    GPIO_NUM_13,      // JTAG PIN
//    GPIO_NUM_34
};
///< all rotary switch pins are pulled up in hardware

const gpio_num_t PIN_LED_CLOCK = GPIO_NUM_19;
const gpio_num_t PIN_LED_DATA_1 = GPIO_NUM_23;
// const gpio_num_t PIN_LED_DATA_2 = GPIO_NUM_25;
// const gpio_num_t PIN_LED_DATA_3 = GPIO_NUM_26;
// const gpio_num_t PIN_LED_DATA_4 = GPIO_NUM_27;
// const gpio_num_t PIN_LED_DATA_5 = GPIO_NUM_32;
// const gpio_num_t PIN_LED_DATA_6 = GPIO_NUM_33;

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

