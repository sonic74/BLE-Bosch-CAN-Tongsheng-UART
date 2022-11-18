// Bluedroid is incompatible with Garmin
#include <NimBLEDevice.h>

// https://github.com/ihaque/pelomon/blob/main/pelomon/ble_constants.h
#include "ble_constants.h"
// https://github.com/ihaque/pelomon/blob/main/pelomon/RideStatus.h
unsigned long last_rpm_timestamp;
unsigned long last_crank_rev_timestamp;
float total_crank_revolutions;
uint16_t current_rpm;
void update_revs_and_time(float& total_revs, unsigned long& event_ts,
                          const float incremental_revs, const float rev_per_ms,
                          const unsigned long ts) {
    const int previous_int_revs = (int) total_revs;
    total_revs += incremental_revs;
    const int integral_revs = (int) total_revs;
    if (previous_int_revs != integral_revs) {
        // Had at least one completed revolution event this time.
        // Need to compute how many ms back we last ticked over a rev
        float partial_revs = total_revs - integral_revs;
        float ms_since_last_cplt_rev = 0;
        if(rev_per_ms!=0.0f) {
          ms_since_last_cplt_rev = partial_revs / rev_per_ms;
        }
        event_ts = ts - (unsigned long) ms_since_last_cplt_rev;
    }
    return;
}

void update_new_rpm(const uint16_t new_rpm) {
    /* Update rpm and total crank revs since last rpm message.
     */
    const unsigned long ts = millis();
    if (last_rpm_timestamp == 0 || (ts - last_rpm_timestamp) > 5000) {
        // Reset our counter if we never saw data or saw it >5s ago
        last_rpm_timestamp = last_crank_rev_timestamp = ts;
        total_crank_revolutions = 0.0f;
    } 
    const unsigned long elapsed_ms = ts - last_rpm_timestamp;
    current_rpm = new_rpm;
    last_rpm_timestamp = ts;
    const float rpmsec_per_rpm = 1.0f / 60000.0f;
    const float rpmsec = rpmsec_per_rpm * current_rpm;
    const float increm_crank_revs = rpmsec * elapsed_ms;
    update_revs_and_time(total_crank_revolutions, last_crank_rev_timestamp,
                         increm_crank_revs, rpmsec, ts);
    if (total_crank_revolutions > 16777215.0f) {
         // Prevent FP32 loss of precision
         // It is ok for crank revolutions to roll over
         total_crank_revolutions -= (unsigned long) total_crank_revolutions;
    }
}

void initialize() {
  current_rpm = 0;
  total_crank_revolutions = 0;
  last_rpm_timestamp = 0;
  last_crank_rev_timestamp = 0;
}


BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;
BLECharacteristic* pCharacteristicBatteryLevel = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;

struct __attribute__((__packed__)) CPSMeasurement_t {
  uint16_t flags;
  uint16_t power; // W
//  uint32_t wheel_revs;
//  uint16_t wheel_rev_timestamp; // 1/2048 s
  uint16_t crank_revs;
  uint16_t crank_rev_timestamp; // 1/1024 s
  //uint16_t energy; // kJ
} CPSMeasurement;

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
#ifdef CANSender_Debug
      Serial.println("onConnect(");
#endif
      deviceConnected = true;
      BLEDevice::startAdvertising();
    };

    void onDisconnect(BLEServer* pServer) {
#ifdef CANSender_Debug
      Serial.println("onDisconnect(");
#endif
      deviceConnected = false;
    }
};

/*class MyCharCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic* pCharacteristic, esp_ble_gatts_cb_param_t* param) {
      Serial.println("onWrite(");
    };

    void onRead(BLECharacteristic* pCharacteristic, esp_ble_gatts_cb_param_t* param) {
      Serial.print("onRead("); Serial.println(pCharacteristic->getUUID().toString().c_str());
    };

    void onNotify(BLECharacteristic* pCharacteristic) {
      Serial.print("onNotify("); Serial.println(pCharacteristic->getUUID().toString().c_str());
    };
};*/


void setup_BLE_server_multiconnect_NimBLE() {
  /*Serial.begin(115200);
  delay(1000); // settle serial*/

  initialize();

  if(ATOMCANBusKit) BLEDevice::init("同盛");
  else BLEDevice::init("Bosch");

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(BLEUUID(/*(uint16_t)ESP_GATT_UUID_CYCLING_POWER_SVC*/CYCLING_POWER_SERVICE_UUID));
  pCharacteristic = pService->createCharacteristic(BLEUUID(CYCLING_POWER_MEASUREMENT_CHAR_UUID), NIMBLE_PROPERTY::NOTIFY);
//  pCharacteristic->setCallbacks(new MyCharCallbacks());
  
  CPSMeasurement.flags=(/*CPM_ACCUMULATED_ENERGY_PRESENT | */CPM_CRANK_REV_DATA_PRESENT);

  BLECharacteristic *pCharCPSFeature = pService->createCharacteristic(BLEUUID(CYCLING_POWER_FEATURE_CHAR_UUID), NIMBLE_PROPERTY::READ);
//  pCharCPSFeature->setCallbacks(new MyCharCallbacks());
  BLECharacteristic *pCharSensor_location = pService->createCharacteristic(BLEUUID(/*(uint16_t)ESP_GATT_UUID_SENSOR_LOCATION*/SENSOR_LOCATION_CHAR_UUID), NIMBLE_PROPERTY::READ);
//  pCharSensor_location->setCallbacks(new MyCharCallbacks());
//  BLECharacteristic *pCharControl_point = pService->createCharacteristic("2A66", NIMBLE_PROPERTY::WRITE|NIMBLE_PROPERTY::INDICATE);
//  pCharControl_point->setCallbacks(new MyCharCallbacks());

  uint32_t pCharCPSFeatureValue=(CPF_CRANK_REVOLUTION_DATA_SUPPORTED | CPF_WHEEL_REVOLUTION_DATA_SUPPORTED/* | CPF_ACCUMULATED_ENERGY_SUPPORTED*/);
  pCharCPSFeature->setValue(pCharCPSFeatureValue);
  uint8_t pCharSensor_locationValue=SENSOR_LOCATION_CHAIN_RING;
  pCharSensor_location->setValue(&pCharSensor_locationValue, sizeof(pCharSensor_locationValue));
/*  uint8_t zero = 0;
  Serial.print("sizeof(zero):"); Serial.println(sizeof(zero));
  pCharControl_point->setValue(&zero, sizeof(zero));*/

  pService->start();

  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(BLEUUID(CYCLING_POWER_SERVICE_UUID));

  uint16_t BATTERY_SERVICE_UUID = (uint16_t)0x180F;
  BLEService *pServiceBattery = pServer->createService(BLEUUID(BATTERY_SERVICE_UUID));
  pCharacteristicBatteryLevel = pServiceBattery->createCharacteristic((uint16_t)0x2A19, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
  /*uint8_t pCharBatteryLevelValue;
  pCharacteristicBatteryLevel->setValue(&pCharBatteryLevelValue, sizeof(pCharBatteryLevelValue));*/
  pServiceBattery->start();
  pAdvertising->addServiceUUID(BLEUUID(BATTERY_SERVICE_UUID));

//  pAdvertising->setScanResponse(false);
//#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(500, UNIT_1_25_MS)            /**< Minimum acceptable connection interval (0.5 seconds). */
//#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(1000, UNIT_1_25_MS)           /**< Maximum acceptable connection interval (1 second). */
//  pAdvertising->setMinPreferred(0x0);  // set value to 0x00 to not advertise this parameter
  pAdvertising->setAppearance(/*ESP_BLE_APPEARANCE_CYCLING_POWER*/0x0484);
  BLEDevice::startAdvertising();

#ifdef CANSender_Debug
  Serial.print("ARDUHAL_LOG_LEVEL="); Serial.println(ARDUHAL_LOG_LEVEL);
  Serial.print("CONFIG_LOG_DEFAULT_LEVEL="); Serial.println(CONFIG_LOG_DEFAULT_LEVEL);
  Serial.print("sizeof(CPSMeasurement):"); Serial.println(sizeof(CPSMeasurement));
  Serial.print("sizeof(pCharCPSFeatureValue):"); Serial.println(sizeof(pCharCPSFeatureValue));
  Serial.print("sizeof(pCharSensor_locationValue):"); Serial.println(sizeof(pCharSensor_locationValue));
  Serial.println("Waiting a client connection to notify...");
#endif
}

//uint16_t sumCadence=0;
void loop_BLE_server_multiconnect_NimBLE() {
    // notify changed value
    if (deviceConnected) {
        int torqueCountTemp=torqueCount;
        if(torqueCountTemp>0) {
          if(ATOMCANBusKit) powerBio=(((float)(torqueSum-78*torqueCount)/(float)torqueCount)*(float)cadence)/(9.55f);
          else powerBio=(((float)torqueSum/(float)torqueCountTemp)*(float)cadence)/(9.55f*10.0f);
          torqueSum=torqueCount=0; // would have to be synchronized
        }
        CPSMeasurement.power=powerBio;
/*        sumCadence+=cadence;
        CPSMeasurement.crank_revs=sumCadence/60;
        CPSMeasurement.crank_rev_timestamp+=1024;*/
        update_new_rpm(cadence);
        CPSMeasurement.crank_revs=(uint16_t) total_crank_revolutions;
        CPSMeasurement.crank_rev_timestamp=last_crank_rev_timestamp;        
        pCharacteristic->setValue(reinterpret_cast<uint8_t*>(&CPSMeasurement), sizeof(CPSMeasurement));
        pCharacteristic->notify();
        
        if(ATOMCANBusKit) pCharacteristicBatteryLevel->setValue((uint8_t)batteryLevel);
        else pCharacteristicBatteryLevel->setValue((uint8_t)batteryLevel); // todo map()
        pCharacteristicBatteryLevel->notify();
        //delay(1000); // bluetooth stack will go into congestion, if too many packets are sent, in 6 hours test i was able to go as low as 3ms
    }
    // disconnecting
    if (!deviceConnected && oldDeviceConnected) {
        //delay(500); // give the bluetooth stack the chance to get things ready
        pServer->startAdvertising(); // restart advertising
#ifdef CANSender_Debug
        Serial.println("start advertising");
#endif
        oldDeviceConnected = deviceConnected;
    }
    // connecting
    if (deviceConnected && !oldDeviceConnected) {
        // do stuff here on connecting
        oldDeviceConnected = deviceConnected;
    }
}
