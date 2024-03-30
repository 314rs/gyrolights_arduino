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

namespace conf 
{
    BLEUUID BLE_SERVICE_UUID(BLE_SERVICE_UUID_str);
    BLEUUID BLE_CHARACTERISTIC_UUID(BLE_CHARACTERISTIC_UUID_str);
}


#if defined(GYRO_MASTER)

extern RotarySwitch<conf::NUM_PINS_ROTARY_SWITCH> test_rotarySwitch;
BLEServer* pServer = nullptr;
BLECharacteristic *pCharacteristic = nullptr;
BLEDescriptor* pDescriptor = new BLE2902();
BLEService *pService = nullptr;
BLEAdvertising *pAdvertising = nullptr;



class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer*) override {
        BLEDevice::startAdvertising(); // allow multiple clients to connect
    };

    void onDisconnect(BLEServer*) override {
    }
};

MyServerCallbacks myservercallbacks;



void startBLE() {
    /**
     * @todo handle pointers savely
     * 
     * @todo everytime this runs, a new service is created. This leads to multiple services
     * also the esp restarts if this is called repeatedly
     * ```
     * [ 29518][D][BLEAdvertising.cpp:199] start(): - advertising service: 018c4715-a90b-7ff1-8c4c-aeed790b0a0a
     * [ 29528][D][BLEAdvertising.cpp:199] start(): - advertising service: 018c4715-a90b-7ff1-8c4c-aeed790b0a0a
     * [ 29538][D][BLEAdvertising.cpp:199] start(): - advertising service: 018c4715-a90b-7ff1-8c4c-aeed790b0a0a
     * [ 29549][D][BLEAdvertising.cpp:199] start(): - advertising service: 018c4715-a90b-7ff1-8c4c-aeed790b0a0a
     * ```
     */

    // Create the BLE Device
    BLEDevice::init(conf::BLE_MASTER_NAME);

    // Create the BLE Server
    if (pServer == nullptr) {
        pServer = BLEDevice::createServer();
        pServer->setCallbacks(&myservercallbacks);
    }

    // Create the BLE Service
    if (pService == nullptr) {
        pService = pServer->createService(conf::BLE_SERVICE_UUID);
    } 

    // Create a BLE Characteristic
    if (pCharacteristic == nullptr) {
        pCharacteristic = pService->createCharacteristic(conf::BLE_CHARACTERISTIC_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_INDICATE);
        // Create a BLE Descriptor
        //pDescriptor->setValue("Oakleaf | Gyro-Mode");  // probably this does nothing
        pCharacteristic->addDescriptor(pDescriptor);
        // Start the service
        pService->start(); /// @bug this throws `[E][BLEDescriptor.cpp:60] executeCreate(): Descriptor already has a handle.` from the second time called
    }    
    uint8_t effect = uint8_t(test_rotarySwitch.getState());
    pCharacteristic->setValue(&effect, 1);



    // Start advertising
    if (pAdvertising == nullptr) {
        pAdvertising = pServer->getAdvertising();
        pAdvertising->addServiceUUID(conf::BLE_SERVICE_UUID);
        pAdvertising->setScanResponse(false);  // server example says `true`, server_multiconnect example says `false` | `true`: services are advertised, `false`: services are not advertised (but tx power is)
        pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue  // set value to 0x00 to not advertise this parameter
        pAdvertising->setMaxPreferred(0x12);  // copied from arduino example. this was `setMinPreferred` again. maybe typo?
    }
    BLEDevice::startAdvertising();
    ESP_LOG_LEVEL(ESP_LOG_INFO, __FUNCTION__, "BLE started.");
}

void stopBLE() {
    BLEDevice::deinit();
}




#elif defined(GYRO_SLAVE)
static BLEScan* pBLEScan = nullptr;
static BLEClient* pClient = nullptr;
static BLEAddress* pServerAddress = nullptr;
static BLERemoteCharacteristic* pRemoteCharacteristic = nullptr;

const uint8_t notificationOn[] = {0x1, 0x0};

//Flags stating if should begin connecting and if the connection is up
static bool doConnect = false;
static bool connected = false;
static bool doScan = false;

TaskHandle_t taskh_slaveBLE = nullptr;
extern esp_event_loop_handle_t loop_handle;
ESP_EVENT_DECLARE_BASE(EFFECT_EVT);
ESP_EVENT_DECLARE_BASE(MODE_EVT);

static void remoteNotifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify) {
    ESP_LOG_LEVEL(ESP_LOG_INFO, __func__, "value: %i", *pData);
    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_event_post_to(loop_handle, EFFECT_EVT, *pData, NULL, 0, 100));
    ///@todo why does this never get called
}


class MyClientCallback : public BLEClientCallbacks {
    void onConnect(BLEClient* pclient) {
        ESP_LOG_LEVEL(ESP_LOG_DEBUG, __func__, "Connected to BLE Device!");
        connected = true;
    }

    void onDisconnect(BLEClient* pclient) {
        //pBLEScan->start(0); // crashes if i remember correctly
        connected = false;
        ESP_LOG_LEVEL(ESP_LOG_WARN, __func__, "Disconnected from to BLE Device!");
        esp_event_post_to(loop_handle, EFFECT_EVT, 0, NULL, 0, 100); /// @todo i dont like that this relies on `conf::effects[0]` to be the staticColor-black function 
    }
};

/**
 * The function `connectToServer` establishes a connection to a remote BLE server, retrieves a specific
 * service and characteristic, and registers for notifications if available.
 * 
 * @param[in] pAddress The `pAddress` parameter in the `connectToServer` function is of type `BLEAddress`
 * and represents the address of the remote BLE server that you want to connect to. This address is
 * used to establish a connection with the specified BLE device.
 * 
 * @retval true if the connection to the server was successful and all necessary characteristics were found and set up correctly
 * @retval false if there was an issue during the connection process or if the required characteristics were not found.
 */
bool connectToServer(BLEAddress pAddress) {
    pClient = BLEDevice::createClient();
    pClient->setClientCallbacks(new MyClientCallback());

    // Connect to the remove BLE Server.
    pClient->connect(pAddress);
    // not needed, since packets are small // pClient->setMTU(517); //set client to request maximum MTU from server (default is 23 otherwise)
    ESP_LOG_LEVEL(ESP_LOG_DEBUG, __func__, "Connected to BLE Device!");

    // Obtain a reference to the service we are after in the remote BLE server.
    BLERemoteService* pRemoteService = pClient->getService(conf::BLE_SERVICE_UUID);
    if (pRemoteService == nullptr) {
        ESP_LOG_LEVEL(ESP_LOG_DEBUG, __func__, "Failed to find our service UUID: %s", conf::BLE_SERVICE_UUID_str);
        pClient->disconnect();
        return false;
    }
    ESP_LOG_LEVEL(ESP_LOG_DEBUG, __func__, "Found service!");
    pRemoteCharacteristic = pRemoteService->getCharacteristic(conf::BLE_CHARACTERISTIC_UUID);
    if(pRemoteCharacteristic->canRead()) {
        uint8_t value = pRemoteCharacteristic->readUInt8();
        ESP_ERROR_CHECK_WITHOUT_ABORT(esp_event_post_to(loop_handle, EFFECT_EVT, value, NULL, 0, 100));
        ESP_LOG_LEVEL(ESP_LOG_VERBOSE, __func__, "The characteristic value is: %d", value);
    }
    ESP_LOG_LEVEL(ESP_LOG_DEBUG, __func__, "Found characteristic!");
    if(pRemoteCharacteristic->canNotify()) {
        pRemoteCharacteristic->registerForNotify(remoteNotifyCallback);
        ESP_LOG_LEVEL(ESP_LOG_VERBOSE, __func__, "registered for notifications");
    } else {
        ESP_LOG_LEVEL(ESP_LOG_ERROR, __func__, "characteristic->canNotify == false");
    }
    //pRemoteCharacteristic->getDescriptor(BLEUUID((uint16_t)0x2902))->writeValue((uint8_t*) notificationOn, 2, true);
    return true;
}

/**
 * Scan for BLE servers and find the first one that advertises the service we are looking for.
 */
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice) {
        if (advertisedDevice.getName() == conf::BLE_MASTER_NAME) 
        // if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(conf::BLE_SERVICE_UUID))  // probably overkill if we assume, there is no ble server withe the same name; also, we need to advertise the services on the master
        {
            advertisedDevice.getScan()->stop();
            if (pServerAddress != nullptr) {
                delete pServerAddress;  //< todo handle memory better
                pServerAddress = nullptr;
            }
            pServerAddress = new BLEAddress(advertisedDevice.getAddress());
            doConnect = true;
            ESP_LOG_LEVEL(ESP_LOG_DEBUG, __func__, "BLE Device Found. Connecting!");
        }
    }
};

void task_slaveBLE(void*) {
    while (true) {
        if (doConnect == true) {
            if (connectToServer(*pServerAddress)) {
                ESP_LOG_LEVEL(ESP_LOG_DEBUG, __func__, "connected");
            }
        doConnect = false;
        }

        if (connected == false) {
            pBLEScan->clearResults();
            pBLEScan->start(0);
            ESP_LOG_LEVEL(ESP_LOG_DEBUG, __func__, "starting scan");
        }
        vTaskDelay(34);
    }
}

void startBLE() {
    // Init BLE
    BLEDevice::init("");
    pBLEScan = BLEDevice::getScan(); //create new scan
    pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
    pBLEScan->setActiveScan(true); //active scan uses more power, but get results faster
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
    BLEDevice::deinit();
}



#endif