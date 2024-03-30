#include <Arduino.h>
#include <esp_log.h>
#include <esp_event.h>
#include <driver/i2c.h>
#include <driver/ledc.h>
#include <lwip/def.h>
#include <ArduinoOTA.h>

#include <FastLED.h>
#include <ESPAsyncE131.h>

#include "button.h"
#include "RotarySwitch.h"

#include "projectConfig.h"
#include "effectsConfig.h"


esp_event_loop_handle_t loop_handle;
ESP_EVENT_DEFINE_BASE(EFFECT_EVT);
ESP_EVENT_DEFINE_BASE(MODE_EVT);

WiFiUDP Udp;
TaskHandle_t task_local = NULL;
TaskHandle_t task_e131 = NULL;
TaskHandle_t taskh_OTA = NULL;

CRGB leds[conf::NUM_STRIPS][conf::NUM_LEDS_PER_STRIP];

static int rotaryswitch = -1;

extern void startBLE();
extern void stopBLE();
extern void startWiFi();
extern void stopWiFi();

#if defined(GYRO_MASTER)
#include <BLEServer.h>
extern BLECharacteristic *pCharacteristic;

RotarySwitch<conf::NUM_PINS_ROTARY_SWITCH> test_rotarySwitch;
#endif
bool switchRF = false;
static button_t rfSwitch;





/**
 * @brief eventloop callback, when effect shall change
 */
void effectChangeCallback(void* hander_arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    ESP_LOG_LEVEL(ESP_LOG_DEBUG, __func__, "event callback fired, event base: %s ; event_id: %d", event_base, event_id);
    rotaryswitch = event_id;  ///< @todo remove

    if (task_local != NULL) {
        ESP_LOG_LEVEL(ESP_LOG_DEBUG, __func__, "delete old task_local");
        vTaskDelete(task_local);
        task_local = NULL;
    }
    ESP_LOG_LEVEL(ESP_LOG_DEBUG, __func__, "create new task_local");
    xTaskCreatePinnedToCore(conf::effects[event_id], "local", configMINIMAL_STACK_SIZE * 16, NULL, 0, &task_local, APP_CPU_NUM);

    // local task darf nicht laufen, wenn im RF Modus
    /// @todo remove hardware dependency
    if (switchRF) {
        ESP_LOG_LEVEL(ESP_LOG_DEBUG, __func__, "suspend task_local, task_local: %p", task_local);
        vTaskSuspend(task_local);
    } else {

#if defined(GYRO_MASTER)
        /// @todo maybe this shouldnt be here. ?
        if (pCharacteristic != nullptr) {
            pCharacteristic->setValue((uint8_t*) &event_id, 1);
            pCharacteristic->notify();
        }
#endif
    }
}

/**
 * @brief eventloop callback, when mode shall change to RF (WiFi and E131)
 */
void modeChangeToRFCallback(void* hander_arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    ESP_LOG_LEVEL(ESP_LOG_DEBUG, __func__, "event callback fired, event base: %s ; event_id: %d", event_base, event_id);
    if (task_local != NULL) {
        vTaskSuspend(task_local);
    }

    fill_solid(*leds, conf::NUM_LEDS_TOTAL, CRGB::Black);
    stopBLE();
    startWiFi();
    switchRF = true;
    FastLED.show();
}

/**
 * @brief eventloop callback, when mode shall change to local (BT, master and slave)
 */
void modeChangeToLocalCallback(void* hander_arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    ESP_LOG_LEVEL(ESP_LOG_DEBUG, __func__, "event callback fired, event base: %s ; event_id: %d", event_base, event_id);
    stopWiFi();
    startBLE();
    switchRF = false;
    fill_solid(*leds, conf::NUM_LEDS_TOTAL, CRGB::Black); 
    FastLED.show(); 
    
    if (task_local != NULL) {
        vTaskResume(task_local);
    } else {
        ESP_ERROR_CHECK_WITHOUT_ABORT(esp_event_post_to(loop_handle, EFFECT_EVT, test_rotarySwitch.getState(), NULL, 0, 100));
    }
}


/**
 * @brief let the onboard led sine-fade
 */
void task_onboardLED(void*) {
    ESP_LOG_LEVEL(ESP_LOG_DEBUG, __FUNCTION__, "%s started", __FUNCTION__);
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



/**
 * @brief callback for the RF switch
 * 
 * the button lib needs this callback
 * @param btn 
 * @param state 
 */
void callbackSwitchRF(button_t *btn, button_state_t state) {
    if (state == BUTTON_PRESSED || state == BUTTON_RELEASED) {
        ESP_LOG_LEVEL(ESP_LOG_DEBUG, __func__, "post event, RF = %d", state);
        ESP_ERROR_CHECK_WITHOUT_ABORT(esp_event_post_to(loop_handle, MODE_EVT, static_cast<int32_t>(state), NULL, 0, 100));
    }
}

#if defined(GYRO_MASTER)
/**
 * RTOS-Style Task. Called in setup, then runs forever
 */
void taskReadRotarySwitch(void*) {
    int prevState = -1;
    while (true) {
        int state = test_rotarySwitch.getState();
        if (state != prevState) {
            prevState = state;
            ESP_LOG_LEVEL(ESP_LOG_DEBUG, __func__, "post event, rotary = %d", state);
            ESP_ERROR_CHECK_WITHOUT_ABORT(esp_event_post_to(loop_handle, EFFECT_EVT, state, NULL, 0, 100));
        }
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}
#endif




void setup() {
    // Serial and debug
    Serial.begin(115200);
    Serial.setDebugOutput(true);

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

#endif
    rfSwitch = button_t{
        .gpio = conf::PIN_RF_SWITCH,
        .internal_pull = false,
        .pressed_level = 0,
        .autorepeat = false,
        .callback = callbackSwitchRF,
    };
    ESP_ERROR_CHECK(button_init(&rfSwitch));
    
    // check state of RF Switch so that the correct mode starts.
    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_event_post_to(loop_handle, MODE_EVT, digitalRead(conf::PIN_RF_SWITCH), NULL, 0, 10));

    // create tasks
    xTaskCreatePinnedToCore(task_onboardLED, "onboard LED", configMINIMAL_STACK_SIZE * 4, NULL, 0, NULL, APP_CPU_NUM);
#if defined(GYRO_MASTER)
    xTaskCreatePinnedToCore(taskReadRotarySwitch, "read rotarySw", configMINIMAL_STACK_SIZE * 8, NULL, 0, NULL, APP_CPU_NUM);
#endif // GYRO_MASTER
    
}

void loop() {
    //ESP_LOGD("loop", "state of e131 task: %d", eTaskGetState(task_e131));
    vTaskDelay(pdMS_TO_TICKS(34));
}
