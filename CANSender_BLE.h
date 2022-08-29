#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

BLEAdvertisementData advert;
BLEAdvertising *pAdvertising;

int i = 0;

//manufacturer code (0x02E5 for Espressif)
int man_code = 0x02E5;

//function takes String and adds manufacturer code at the beginning 
void setManData(String c, int c_size, BLEAdvertisementData &adv, int m_code) {
  
  String s;
  char b2 = (char)(m_code >> 8);
  m_code <<= 8;
  char b1 = (char)(m_code >> 8);
  s.concat(b1);
  s.concat(b2);
  s.concat(c);
  adv.setManufacturerData(s.c_str());
}

void setup_BLE() {
//  Serial.begin(115200);
  Serial.println("Starting BLE work!");

  BLEDevice::init("MyESP32");
  BLEServer *pServer = BLEDevice::createServer();

  pAdvertising = pServer->getAdvertising();
  advert.setName("M5AtomCAN");
  pAdvertising->setAdvertisementData(advert);
  pAdvertising->start();
}

void loop_BLE(char* zk) {
  String a = String(zk);
  
  BLEAdvertisementData scan_response;
  setManData(a, a.length() , scan_response, man_code);

  pAdvertising->stop();
  pAdvertising->setScanResponseData(scan_response);
  pAdvertising->start();
}
