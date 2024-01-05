#include <Arduino.h>
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
//BLEDescriptor* pDescriptor = new BLE2902();
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
    //pDescriptor->setValue("Oakleaf | Gyro-Mode");
    //pCharacteristic->addDescriptor(pDescriptor);
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
#endif