#include <Arduino.h>
#include <esp_log.h>
#include <driver/i2c.h>
#include <driver/ledc.h>
#include <lwip/def.h>
#include <ArduinoOTA.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

#if defined(GYRO_MASTER)
    #include <BLE2902.h>
#elif defined(GYRO_SLAVE)
    #include <BLEAdvertisedDevice.h>
#endif


#include <FastLED.h>
//#include <ESPAsyncE131.h>
//#include <ESPTelnet.h>

#include "button.h"

#include "projectConfig.h"


WiFiUDP Udp;
TaskHandle_t task_local = NULL;

#if defined(GYRO_MASTER)
TaskHandle_t task_e131 = NULL;

static portMUX_TYPE my_spinlock = portMUX_INITIALIZER_UNLOCKED;

WiFiServer Server(23);

static SemaphoreHandle_t stateMutex;
bool switchRF = false;

static button_t rotarySwitch[conf::NUM_PINS_ROTARY_SWITCH];
static button_t rfSwitch;

BLECharacteristic *pCharacteristic = nullptr;
BLEDescriptor* pDescriptor = new BLE2902();
static bool deviceConnected = false;

#elif defined(GYRO_SLAVE)
static BLEScan* pBLEScan;
static BLEClient* pClient;
static BLEAddress* pServerAddress;
static BLERemoteCharacteristic* pRemoteCharacteristic;

const uint8_t notificationOn[] = {0x1, 0x0};

//Flags stating if should begin connecting and if the connection is up
static boolean doConnect = false;
static boolean connected = false;
#endif


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



#if defined(GYRO_MASTER)


/* void telnetTask(void*) {
    while (true) {
        telnet.loop();
        if (Serial.available()) {
            telnet.print(Serial.read());
        }
    }
};

int telnetLogCallback(const char *fmt, va_list args) {
    if (telnet.isConnected()) {
        char buf[256];
        vsnprintf(buf, sizeof(buf), fmt, args);
        telnet.print(buf);
    }
    return ESP_OK;
}

void onTelnetInput(String str) {
    // checks for a certain command
    if (str == "ping") {
        telnet.println("> pong");
    // disconnect the client
    } else if (str == "bye" || str == "quit") {
        telnet.println("> disconnecting you...");
        telnet.disconnectClient();
    }
    ESP_LOG_LEVEL(ESP_LOG_INFO, "telnet", "received: %s", str.c_str());
} */



/**
 * @brief task to handle e131 when active
 * 
 */
/* void e131task(void*) {
    esp_log_write(ESP_LOG_DEBUG, __FUNCTION__, "%s started", __FUNCTION__);
    uint16_t i = 0;
    while (true)
    {
        if (!e131.isEmpty()) {
            e131_packet_t packet;
            e131.pull(&packet);
            CRGB color = CRGB(packet.property_values[1],packet.property_values[2],packet.property_values[3]);
            fill_solid(*leds, conf::NUM_LEDS_TOTAL, color);
            FastLED.show();
            esp_log_write(ESP_LOG_DEBUG, __func__, "got color: %x", color);
        }
        vTaskDelay(1);
        if (i == 0) {
        }
        esp_log_write(ESP_LOG_DEBUG, __FUNCTION__, "high watermark: %d", uxTaskGetStackHighWaterMark(NULL));
        i++;
    }
}; */

/**
 * @brief 
 * 
 * @param fmt 
 * @param args 
 * @return `ESP_OK` on success
 */
int udpLogCallback(const char *fmt, va_list args) {
    static WiFiClient client;
    char buf[256];
    if (Server.hasClient()) {
        if (client.connected()) {
            Server.available().stop();
        } else {

        client = Server.available();
        }
    }
        vsnprintf(buf, sizeof(buf), fmt, args);
        client.print(buf);
    return ESP_OK;
}


/**
 * @brief provide OTA Update functionality
 * 
 */
void otaFun(void*) {
    esp_log_write(ESP_LOG_DEBUG, __FUNCTION__, "%s started", __FUNCTION__);
    ArduinoOTA.begin();
    while (true) {
        ArduinoOTA.handle();
        vTaskDelay(10);
    }
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
 * @brief callback for the RF switch
 * 
 * @param btn 
 * @param state 
 */
void callbackSwitchRF(button_t *btn, button_state_t state) {
    if (state == BUTTON_PRESSED) {
        switchRF = true;
        if (task_e131 != NULL) {
            vTaskResume(task_e131);
            ESP_LOGI("switchRF", "RF on");
        }
        fill_solid(*leds, conf::NUM_LEDS_TOTAL, CRGB::Black);
        FastLED.show();
        if (task_local != NULL) {
            vTaskSuspend(task_local);
        }
    } else if (state == BUTTON_RELEASED) {
        switchRF = false;
        if (task_e131 != NULL) {
            vTaskSuspend(task_e131);
            ESP_LOGI("switchRF", "RF off");
        }
        fill_solid(*leds, conf::NUM_LEDS_TOTAL, CRGB::Black);
        FastLED.show();
        if (task_local != NULL) {
            vTaskResume(task_local);
        }
    }
}

/**
 * @brief callback for the rotary switch
 * 
 * @param btn 
 * @param state 
 */
void callbackRotaryswitch(button_t *btn, button_state_t state) {
    if (state == BUTTON_PRESSED) {
        rotaryswitch = (btn - rotarySwitch);
        ESP_LOGI("rotarySwitch", "rotarySwitch position %d", rotaryswitch);
        if (pCharacteristic != nullptr) 
            pCharacteristic->setValue((uint8_t*) &rotaryswitch, 1);
            pCharacteristic->notify();
        if (switchRF) {
            if (task_local != NULL)
                vTaskSuspend(task_local);
        } else {
            if (task_local != NULL)
                vTaskResume(task_local);
        }
    }
}

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      BLEDevice::startAdvertising();
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};


#elif defined(GYRO_SLAVE)
static void remoteNotifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify) {
    ESP_LOG_LEVEL(ESP_LOG_INFO, __func__, "value: %i", *pData);
    rotaryswitch = *pData;
}


class MyClientCallback : public BLEClientCallbacks {
    void onConnect(BLEClient* pclient) {}

    void onDisconnect(BLEClient* pclient) {
        ESP_LOG_LEVEL(ESP_LOG_WARN, __func__, "Disconnected from to BLE Device!");
        //pBLEScan->start(0);z
        ESP.restart();
        connected = false;
    }
};

bool connectToServer(BLEAddress pAddress) {
    pClient = BLEDevice::createClient();
    pClient->setClientCallbacks(new MyClientCallback());

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
        if (advertisedDevice.getName() == conf::BLE_MASTER_NAME) {
            advertisedDevice.getScan()->stop();
            pServerAddress = new BLEAddress(advertisedDevice.getAddress());
            ESP_LOG_LEVEL(ESP_LOG_DEBUG, __func__, "BLE Device Found. Connecting!");
            doConnect = true;
        }
    }
};


#endif


#if defined(GYRO_MASTER)




/**
 * @brief Start WiFi and also e131 operation mode.
 * 
 */
void startWiFi() {
    // AP
    char apName[32];
    uint8_t mac[16];
    esp_read_mac(mac, esp_mac_type_t::ESP_MAC_WIFI_STA);
    sprintf(apName, "%s%X%X%X", conf::WIFI_AP_SSID_PREFIX, mac[3], mac[4], mac[5]);
    ESP_LOGI("mac","%s", apName);
    WiFi.mode(WIFI_AP);
    WiFi.softAP(apName, conf::WIFI_AP_PW);
}

void stopWifi() {

}

void startBLE() {
    BLEDevice::init(conf::BLE_MASTER_NAME);
    BLEServer *pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());
    BLEService *pService = pServer->createService(conf::BLE_SERVICE_UUID);
    pCharacteristic = pService->createCharacteristic(conf::BLE_CHARACTERISTIC_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
    pCharacteristic->setValue("Value: Hello World!");
    pDescriptor->setValue("Oakleaf | Gyro-Mode");
    pCharacteristic->addDescriptor(pDescriptor);
    pService->start();
    BLEAdvertising *pAdvertising = pServer->getAdvertising();
    pAdvertising->addServiceUUID(conf::BLE_SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x12);
    BLEDevice::startAdvertising();
    esp_log_write(ESP_LOG_INFO, __FUNCTION__, "BLE started.");
}

void stopBLE() {

}

#endif

void setup() {
    // Serial and debug
    Serial.begin(115200);
    Serial.setDebugOutput(true);
    //esp_log_set_vprintf(telnetLogCallback); // route log output to telnet
    esp_log_level_set("*", ESP_LOG_DEBUG);
    //esp_log_level_set("printGyro", ESP_LOG_ERROR);

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

    
    // e131
    delay(100);
    //ESP_ERROR_CHECK(!e131.begin(E131_MULTICAST, conf::UNIVERSE, conf::UNIVERSE_COUNT));
   


    


    // FastLED
    // black
    FastLED.addLeds<APA102, conf::PIN_LED_DATA_1, conf::PIN_LED_CLOCK,  BGR>(leds[0], conf::NUM_LEDS_PER_STRIP);
    
    
    fill_solid(*leds, conf::NUM_LEDS_TOTAL, CRGB::Black);
    FastLED.show();

    // sanity delay
    delay(1000);

#if defined(GYRO_MASTER)
    startBLE();

     // init input
    for (int i = 0; i < conf::NUM_PINS_ROTARY_SWITCH; i++) {
        rotarySwitch[i] = button_t{
            .gpio = conf::PINS_ROTARY_SWITCH[i],
            .internal_pull = false,
            .pressed_level = 0,
            .autorepeat = false,
            .callback = callbackRotaryswitch,
        };
        ESP_ERROR_CHECK(button_init(&rotarySwitch[i]));
    }
    rfSwitch = button_t{
        .gpio = conf::PIN_RF_SWITCH,
        .internal_pull = false,
        .pressed_level = 0,
        .autorepeat = false,
        .callback = callbackSwitchRF,
    };
    ESP_ERROR_CHECK(button_init(&rfSwitch));

#elif defined(GYRO_SLAVE)
     // Init BLE
    BLEDevice::init("");
    pBLEScan = BLEDevice::getScan();
    pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
    pBLEScan->setActiveScan(true);
    pBLEScan->start(0);
#endif // GYRO_MASTER
    
   

    // create tasks
    //xTaskCreatePinnedToCore(telnetTask, "telnet task", configMINIMAL_STACK_SIZE * 4, NULL, 1, NULL, APP_CPU_NUM);
    //xTaskCreatePinnedToCore(otaFun, "OTAFun", configMINIMAL_STACK_SIZE * 4, NULL, 0, NULL, APP_CPU_NUM);
    xTaskCreatePinnedToCore(ledFun, "buildin LED", configMINIMAL_STACK_SIZE * 4, NULL, 0, NULL, APP_CPU_NUM);
    //xTaskCreatePinnedToCore(e131task, "e131", configMINIMAL_STACK_SIZE * 16, NULL, 1, &task_e131,APP_CPU_NUM);
    //vTaskSuspend(task_e131);
    xTaskCreatePinnedToCore(localtask, "local", configMINIMAL_STACK_SIZE * 16, NULL, 2, &task_local, APP_CPU_NUM);
    //vTaskSuspend(task_local);
}

#if defined(GYRO_MASTER)
void loop() {
    //ESP_LOGD("loop", "state of e131 task: %d", eTaskGetState(task_e131));
    vTaskDelay(pdMS_TO_TICKS(34));
    //FastLED.show(conf::MAX_BRIGHTNESS);
}

#elif defined(GYRO_SLAVE)
void loop() {
    if (doConnect == true) {
        if (connectToServer(*pServerAddress)) {
            ESP_LOG_LEVEL(ESP_LOG_DEBUG, __func__, "connected");
            connected = true;
        }
        doConnect = false;
    }

    if (connected = false) {
        pBLEScan->start(0);
    }


    vTaskDelay(10);
}
#endif 
