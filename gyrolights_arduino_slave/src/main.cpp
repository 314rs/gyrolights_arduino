#include <Arduino.h>
#include <esp_log.h>
#include <driver/i2c.h>
#include <driver/ledc.h>
#include <lwip/def.h>
#include <ArduinoOTA.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLEAdvertisedDevice.h>

#include <FastLED.h>
#include <ESPAsyncE131.h>
#include <ESPTelnet.h>



#include "button.h"

#include "../../gyrolights_arduino_master/include/projectConfig.h"

static BLEScan* pBLEScan;
static BLEClient* pClient;
static BLEAddress* pServerAddress;
static BLERemoteCharacteristic* pRemoteCharacteristic;

static CRGB leds[conf::NUM_STRIPS][conf::NUM_LEDS_PER_STRIP];

TaskHandle_t task_local = NULL;

static int8_t rotaryswitch = -1;

const uint8_t notificationOn[] = {0x1, 0x0};

//Flags stating if should begin connecting and if the connection is up
static boolean doConnect = false;
static boolean connected = false;

static void remoteNotifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify) {
    ESP_LOG_LEVEL(ESP_LOG_INFO, __func__, "value: %i", *pData);
    rotaryswitch = *pData;
}

bool connectToServer(BLEAddress pAddress) {
    pClient = BLEDevice::createClient();

    pClient->connect(pAddress);
    ESP_LOG_LEVEL(ESP_LOG_DEBUG, __func__, "Connected to BLE Device!");
    BLERemoteService* pRemoteService = pClient->getService(conf::BLE_SERVICE_UUID);
    ESP_LOG_LEVEL(ESP_LOG_DEBUG, __func__, "Found service!");
    pRemoteCharacteristic = pRemoteService->getCharacteristic(conf::BLE_CHARACTERISTIC_UUID);
    ESP_LOG_LEVEL(ESP_LOG_DEBUG, __func__, "Found characteristic!");
    pRemoteCharacteristic->registerForNotify(remoteNotifyCallback);
    //pRemoteCharacteristic->getDescriptor(BLEUUID((uint16_t)0x2902))->writeValue((uint8_t*) notificationOn, 2, true);
    return true;
}

class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice) {
        if (advertisedDevice.getName() == "ESP32") {
            advertisedDevice.getScan()->stop();
            pServerAddress = new BLEAddress(advertisedDevice.getAddress());
            ESP_LOG_LEVEL(ESP_LOG_DEBUG, __func__, "BLE Device Found. Connecting!");
            doConnect = true;
        }
    }
};

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
        ESP_LOG_LEVEL(ESP_LOG_DEBUG, __FUNCTION__, "high watermark: %d, rotaryswitch: %d", uxTaskGetStackHighWaterMark(NULL), rotaryswitch);
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
            fill_solid(*leds, conf::NUM_LEDS_TOTAL, CRGB::Cyan);
            readGyro(rxvalues);
            ESP_LOG_LEVEL(ESP_LOG_INFO, "readGyro", "accelx: %7d, accely: %7d, accelz: %7d, temp: %7d, gyrox: %7d, gyroy: %7d, gyroz: %7d\r", (rxvalues[0]), (rxvalues[1]), (rxvalues[2]), (rxvalues[3]), (rxvalues[4]), (rxvalues[5]), (rxvalues[6]));

            //FastLED.show(conf::MAX_BRIGHTNESS);
            //vTaskSuspend(NULL);
            break;
        case 3:
            directednormaccel = directedAccelNorm();
            if (directednormaccel < -10000) {
                fill_solid(*leds, conf::NUM_LEDS_TOTAL, CRGB::Pink);
            } else {
                fill_solid(*leds, conf::NUM_LEDS_TOTAL, CRGB::Black);
            }
            ESP_LOG_LEVEL(ESP_LOG_INFO, "accel", "acceleration norm: %f\r", directednormaccel);
            //FastLED.show(conf::MAX_BRIGHTNESS);
            //vTaskSuspend(NULL);
            break;
        case 4:
            fill_solid(*leds, conf::NUM_LEDS_TOTAL, CRGB::SeaGreen);
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
            fill_solid(*leds, conf::NUM_LEDS_TOTAL, CRGB::SeaGreen);
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




void setup() {
    // Serial and debug
    Serial.begin(115200);
    Serial.setDebugOutput(true);
    esp_log_level_set("*", ESP_LOG_DEBUG);

    // config mpu6050 gyro sensor
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = conf::PIN_MPU6050_SDA,
        .scl_io_num = conf::PIN_MPU6050_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master{.clk_speed = 400000}
    };
    i2c_param_config(I2C_NUM_0, &conf);
    ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));
    static const uint8_t setuptx[] = {0x6B, 0x00}; // wakeup
    ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_master_write_to_device(I2C_NUM_0, conf::I2C_GYRO_ADDR, setuptx, sizeof(setuptx), conf::I2C_TIMEOUT_MS/portTICK_RATE_MS));
    static const uint8_t acc_range[] = {0x1C, (conf::GYRO_ACCEL_SENSITIVITY << 3)}; // range of accelerometer   // register for gyro: 0x1b
    ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_master_write_to_device(I2C_NUM_0, conf::I2C_GYRO_ADDR, acc_range, sizeof(acc_range), conf::I2C_TIMEOUT_MS/portTICK_RATE_MS));


    // Init BLE
    BLEDevice::init("");
    pBLEScan = BLEDevice::getScan();
    pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
    pBLEScan->setActiveScan(true);
    pBLEScan->start(30);

    //FastLED
    FastLED.addLeds<APA102, conf::PIN_LED_DATA_5, conf::PIN_LED_CLOCK,  BGR>(leds[0], conf::NUM_LEDS_PER_STRIP);


    xTaskCreatePinnedToCore(ledFun, "buildin LED", configMINIMAL_STACK_SIZE * 4, NULL, 0, NULL, APP_CPU_NUM);

    xTaskCreatePinnedToCore(localtask, "local", configMINIMAL_STACK_SIZE * 16, NULL, 2, &task_local, APP_CPU_NUM);

}



void loop() {
    if (doConnect == true) {
        if (connectToServer(*pServerAddress)) {
            ESP_LOG_LEVEL(ESP_LOG_DEBUG, __func__, "connected");
        }
        doConnect = false;
    }


    vTaskDelay(10);
}