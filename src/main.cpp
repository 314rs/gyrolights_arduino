#include <Arduino.h>
#include <esp_log.h>
#include <esp_event.h>
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
#include <ESPAsyncE131.h>
#include <ESPTelnet.h>

#include "button.h"
#include "RotarySwitch.h"

#include "projectConfig.h"
#include "effectsConfig.h"


esp_event_loop_handle_t loop_handle;
ESP_EVENT_DEFINE_BASE(EFFECT_EVT);
ESP_EVENT_DEFINE_BASE(MODE_EVT);

WiFiUDP Udp;
ESPTelnet telnet;
ESPAsyncE131 e131(conf::UNIVERSE_COUNT);
TaskHandle_t task_local = NULL;

CRGB leds[conf::NUM_STRIPS][conf::NUM_LEDS_PER_STRIP];

static int8_t rotaryswitch = -1;


#if defined(GYRO_MASTER)
TaskHandle_t task_e131 = NULL;


WiFiServer Server(23);

static RotarySwitch<conf::NUM_PINS_ROTARY_SWITCH> test_rotarySwitch;

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

    //ESP_ERROR_CHECK(!e131.begin(E131_MULTICAST, conf::UNIVERSE, conf::UNIVERSE_COUNT));
        //xTaskCreatePinnedToCore(e131task, "e131", configMINIMAL_STACK_SIZE * 16, NULL, 1, &task_e131, APP_CPU_NUM);
        //esp_log_set_vprintf(telnetLogCallback); // route log output to telnet

}

void stopWifi() {

    //vTaskSuspend(task_e131);
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


/**
 * @brief eventloop callback, when effect shall change
 */
void effectChangeCallback(void* hander_arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    ESP_LOG_LEVEL(ESP_LOG_DEBUG, __func__, "event callback fired, event base: %s ; event_id: %d", event_base, event_id);
    rotaryswitch = event_id;  ///< @todo remove

    if (task_local != NULL) {
        vTaskDelete(task_local);
        task_local = NULL;
    }
    xTaskCreatePinnedToCore(conf::effects[event_id], "local", configMINIMAL_STACK_SIZE * 16, NULL, 2, &task_local, APP_CPU_NUM);
    
    // local task darf nicht laufen, wenn im RF Modus
    /// @todo remove hardware dependency
    if (switchRF) {
        vTaskSuspend(task_local);
    } 


    /// @todo maybe this shouldnt be here. ?
    /* #if defined(GYRO_MASTER)
    if (pCharacteristic != nullptr) 
            pCharacteristic->setValue((uint8_t*) &event_id, 1);
            pCharacteristic->notify();
    #endif */

}

/**
 * @brief eventloop callback, when mode shall change to RF (WiFi and E131)
 */
void modeChangeToRFCallback(void* hander_arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    ESP_LOG_LEVEL(ESP_LOG_DEBUG, __func__, "event callback fired, event base: %s ; event_id: %d", event_base, event_id);

    //stopBLE();
    //startWiFi();

    switchRF = true;
    if (task_e131 != NULL) {
        vTaskResume(task_e131);
        ESP_LOGI("switchRF", "RF on");
    } else {
        // xTaskCreatePinnedToCore(e131task, "e131", configMINIMAL_STACK_SIZE * 16, NULL, 2, &task_local, APP_CPU_NUM); /// @todo check implementation
    }
    fill_solid(*leds, conf::NUM_LEDS_TOTAL, CRGB::Black);
    FastLED.show();
    if (task_local != NULL) {
        vTaskSuspend(task_local);
    }
}

/**
 * @brief eventloop callback, when mode shall change to local (BT, master and slave)
 */
void modeChangeToLocalCallback(void* hander_arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    ESP_LOG_LEVEL(ESP_LOG_DEBUG, __func__, "event callback fired, event base: %s ; event_id: %d", event_base, event_id);

    //stopWifi();
    //startBLE();

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



#if defined(GYRO_MASTER)



/**
 * @brief callback for the RF switch
 * 
 * the button lib needs this callback
 * @param btn 
 * @param state 
 */
void callbackSwitchRF(button_t *btn, button_state_t state) {
    if (state == BUTTON_PRESSED || state == BUTTON_RELEASED) {
        esp_event_post_to(loop_handle, MODE_EVT, static_cast<int32_t>(state), NULL, 0, 0);
    }
}


void taskReadRotarySwitch(void*) {
    int prevState = -1;
    while (true) {
        int state = test_rotarySwitch.getState();
        if (state != prevState) {
            prevState = state;
            esp_event_post_to(loop_handle, EFFECT_EVT, state, NULL, 0, 0);
        }
    }
}





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




void setup() {
    // Serial and debug
    Serial.begin(115200);
    Serial.setDebugOutput(true);
    esp_log_level_set("*", ESP_LOG_DEBUG);

    // create input event loop
    esp_event_loop_args_t loop_args = {
        .queue_size = 10,
        .task_name = "input loop task",
        .task_priority = 1,
        .task_stack_size = 0x1000,
        .task_core_id = tskNO_AFFINITY
    };
    esp_event_loop_create(&loop_args, &loop_handle);
    esp_event_handler_register_with(loop_handle, MODE_EVT, BUTTON_PRESSED, modeChangeToRFCallback, NULL); ///< @todo maybe button states are mixed up.
    esp_event_handler_register_with(loop_handle, MODE_EVT, BUTTON_RELEASED, modeChangeToLocalCallback, NULL);
    esp_event_handler_register_with(loop_handle, EFFECT_EVT, ESP_EVENT_ANY_ID, effectChangeCallback, NULL);

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

    // FastLED
    FastLED.addLeds<APA102, conf::PIN_LED_DATA_1, conf::PIN_LED_CLOCK, BGR>(leds[0], conf::NUM_LEDS_PER_STRIP);
    fill_solid(*leds, conf::NUM_LEDS_TOTAL, CRGB::Black);
    FastLED.show();

    // sanity delay
    delay(100);

#if defined(GYRO_MASTER)

     // init input
    test_rotarySwitch.init(conf::PINS_ROTARY_SWITCH, 40000);

    rfSwitch = button_t{
        .gpio = conf::PIN_RF_SWITCH,
        .internal_pull = false,
        .pressed_level = 0,
        .autorepeat = false,
        .callback = callbackSwitchRF,
    };
    ESP_ERROR_CHECK(button_init(&rfSwitch));

    // ble, maybe not neccessary
    startBLE();

#elif defined(GYRO_SLAVE)
     // Init BLE
    BLEDevice::init("");
    pBLEScan = BLEDevice::getScan();
    pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
    pBLEScan->setActiveScan(true);
    pBLEScan->start(0);
#endif
    
   

    // create tasks
    //xTaskCreatePinnedToCore(telnetTask, "telnet task", configMINIMAL_STACK_SIZE * 4, NULL, 1, NULL, APP_CPU_NUM);
    //xTaskCreatePinnedToCore(otaFun, "OTAFun", configMINIMAL_STACK_SIZE * 4, NULL, 0, NULL, APP_CPU_NUM);
    xTaskCreatePinnedToCore(ledFun, "buildin LED", configMINIMAL_STACK_SIZE * 4, NULL, 0, NULL, APP_CPU_NUM);
    xTaskCreatePinnedToCore(taskReadRotarySwitch, "read rotarySw", configMINIMAL_STACK_SIZE * 8, NULL, 0, NULL, APP_CPU_NUM);
    //xTaskCreatePinnedToCore(localtask, "local", configMINIMAL_STACK_SIZE * 16, NULL, 2, &task_local, APP_CPU_NUM);
}

void loop() {

#if defined(GYRO_MASTER)

    //ESP_LOGD("loop", "state of e131 task: %d", eTaskGetState(task_e131));
    vTaskDelay(pdMS_TO_TICKS(34));
    //FastLED.show(conf::MAX_BRIGHTNESS);

#elif defined(GYRO_SLAVE)

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

#endif 

}



















void telnetTask(void*) {
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
}



/**
 * @brief task to handle e131 when active
 * 
 */
void e131task(void*) {
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
}; 


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


