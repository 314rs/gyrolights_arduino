#include <Arduino.h>
#include <esp_err.h>
#include <esp_event.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#if defined(GYRO_MASTER)
#include <BLE2902.h>
#elif defined(GYRO_SLAVE)
#include <BLEAdvertisedDevice.h>
#endif

#include "RotarySwitch.h"

#include "projectConfig.h"



#if defined(GYRO_MASTER)

extern RotarySwitch<conf::NUM_PINS_ROTARY_SWITCH> test_rotarySwitch;
BLECharacteristic *pCharacteristic = nullptr;
BLEDescriptor* pDescriptor = new BLE2902();
static bool deviceConnected = false;



class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      BLEDevice::startAdvertising();
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};

MyServerCallbacks myservercallbacks;



void startBLE() {
    BLEDevice::init(conf::BLE_MASTER_NAME);
    BLEServer *pServer = BLEDevice::createServer();
    pServer->setCallbacks(&myservercallbacks);
    BLEService *pService = pServer->createService(conf::BLE_SERVICE_UUID);
    pCharacteristic = pService->createCharacteristic(conf::BLE_CHARACTERISTIC_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
    uint8_t effect = test_rotarySwitch.getState();
    pCharacteristic->setValue(&effect, 1);
    pDescriptor->setValue("Oakleaf | Gyro-Mode");
    pCharacteristic->addDescriptor(pDescriptor);
    pService->start();
    BLEAdvertising *pAdvertising = pServer->getAdvertising();
    pAdvertising->addServiceUUID(conf::BLE_SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x12);
    BLEDevice::startAdvertising();
    ESP_LOG_LEVEL(ESP_LOG_INFO, __FUNCTION__, "BLE started.");
}

void stopBLE() {
    BLEDevice::deinit();
}




#elif defined(GYRO_SLAVE)
static BLEScan* pBLEScan;
static BLEClient* pClient;
static BLEAddress* pServerAddress;
static BLERemoteCharacteristic* pRemoteCharacteristic;

const uint8_t notificationOn[] = {0x1, 0x0};

//Flags stating if should begin connecting and if the connection is up
static boolean doConnect = false;
static boolean connected = false;

TaskHandle_t taskh_slaveBLE = NULL;
extern esp_event_loop_handle_t loop_handle;
ESP_EVENT_DECLARE_BASE(EFFECT_EVT);
ESP_EVENT_DECLARE_BASE(MODE_EVT);

static void remoteNotifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify) {
    ESP_LOG_LEVEL(ESP_LOG_INFO, __func__, "value: %i", *pData);
    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_event_post_to(loop_handle, EFFECT_EVT, *pData, NULL, 0, 100));
    ///@todo why does this never get called
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

void task_slaveBLE(void*) {
    while (true) {
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
        vTaskDelay(34);
    }
}

void startBLE() {
    // Init BLE
    BLEDevice::init("");
    pBLEScan = BLEDevice::getScan();
    pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
    pBLEScan->setActiveScan(true);
    pBLEScan->start(0);
    if (taskh_slaveBLE == NULL) {
        xTaskCreatePinnedToCore(task_slaveBLE, "slaveBLE", configMINIMAL_STACK_SIZE * 4, NULL, 0, NULL, APP_CPU_NUM);
    } else {
        vTaskResume(taskh_slaveBLE);
    }
}


void stopBLE() {
    if (taskh_slaveBLE != NULL) {
        vTaskSuspend(taskh_slaveBLE);
    }
}



#endif