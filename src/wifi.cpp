#include <Arduino.h>
#include <WiFi.h>
#include <esp_err.h>
#include <ArduinoOTA.h>

#include <FastLED.h>
#include <ESPAsyncE131.h>

#include "projectConfig.h"


extern CRGB** leds;
extern TaskHandle_t task_e131;
extern TaskHandle_t taskh_OTA;
ESPAsyncE131 e131(conf::UNIVERSE_COUNT);


/**
 * @brief task to handle e131 when active
 * 
 */
void e131task(void*) {
    ESP_LOG_LEVEL(ESP_LOG_DEBUG, __FUNCTION__, "%s started", __FUNCTION__);
    while (true)
    {
        if (!e131.isEmpty()) {
            e131_packet_t packet;
            e131.pull(&packet);
            CRGB color = CRGB(packet.property_values[1],packet.property_values[2],packet.property_values[3]);
            fill_solid(*leds, conf::NUM_LEDS_TOTAL, color);
            FastLED.show();
            ESP_LOG_LEVEL(ESP_LOG_DEBUG, __func__, "got color: %x", color);
        }
        vTaskDelay(34);
        ESP_LOG_LEVEL(ESP_LOG_VERBOSE, __FUNCTION__, "high watermark: %d", uxTaskGetStackHighWaterMark(NULL));
    }
}

/**
 * @brief provide OTA Update functionality
 * 
 */
void task_OTA(void*) {
    ESP_LOGD(__func__, "%s started", __func__);
    ArduinoOTA.begin();
    while (true) {
        ArduinoOTA.handle();
        vTaskDelay(34);
    }
}



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
    ESP_LOGI(__func__,"mac: %s", apName);
    WiFi.mode(WIFI_AP);
    WiFi.softAP(apName, conf::WIFI_AP_PW);
    IPAddress ipaddr = WiFi.softAPIP();
    ESP_LOGD(__func__, "wifi started. IP-Addr: %s", ipaddr.toString().c_str());

    ESP_ERROR_CHECK_WITHOUT_ABORT(!e131.begin(E131_MULTICAST, conf::UNIVERSE, conf::UNIVERSE_COUNT));
    if (task_e131 == NULL) {
        ESP_LOGD(__func__, "create task_e131");
        xTaskCreatePinnedToCore(e131task, "e131", configMINIMAL_STACK_SIZE * 16, NULL, 0, &task_e131, APP_CPU_NUM);
    } else {
        vTaskResume(task_e131);
    }
    if (taskh_OTA == NULL) {
        ESP_LOGD(__func__, "create task_OTA");
        xTaskCreatePinnedToCore(task_OTA, "OTA", configMINIMAL_STACK_SIZE * 4, NULL, 0, &taskh_OTA, APP_CPU_NUM);
    } else {
        vTaskResume(taskh_OTA);
    }


    //esp_log_set_vprintf(telnetLogCallback); // route log output to telnet

}

void stopWiFi() {
    if (task_e131 != NULL ) {
        ESP_LOG_LEVEL(ESP_LOG_DEBUG, __func__, "suspend task_e131");
        vTaskSuspend(task_e131);
    }
    if (taskh_OTA != NULL ) {
        ESP_LOG_LEVEL(ESP_LOG_DEBUG, __func__, "suspend taskh_OTA");
        vTaskSuspend(taskh_OTA);
    }
    WiFi.disconnect();
    WiFi.mode(WIFI_OFF);
}