/* Bosch CAN PowerPack activation and Bosch & Tongsheng TSDZ2 Garmin compatible Bluetooth LE Cycling Power Profile
 *
 * works with esp32 2.0.4
 * worked with M5Stack 2.0.3 & M5Atom 0.0.8 (esp32 1.0.6)
 *
 * Copyright 2021-2022 Sven Killig (sven@killig.de)
 * http://sven.killig.de/Bosch/CANSender/
 */

// https://github.com/adafruit/arduino-CAN
#include <CAN.h>

bool ATOMCANBusKit=false;
//#define CANSender_Debug
// ################################################################################################################################

bool tx = false;

int16_t current, power, voltage;
int temperature = 27315;

unsigned char batteryLevel, motorStatus, /*torqueTara, torqueActual*/ cadence, errorCode;
int16_t torque;
int torqueSum, torqueCount;
unsigned int speedSensor;

int16_t powerBio;

unsigned char motorControl;

union bytes2int_t {
  uint8_t myBytes[2];
  int16_t myInt;
} bytes2int;


//#define CANSender_BLE
#define BLE_server_multiconnect_NimBLE
#if defined(CANSender_BLE)
#include "CANSender_BLE.h"
#elif defined(BLE_server_multiconnect_NimBLE)
#include "BLE_server_multiconnect_NimBLE.h"
#endif

#ifdef ARDUINO_TBeam
#include "CANSender_TBeam.h"
#endif

#ifdef ARDUINO_M5Stack_ATOM
#include "CANSender_M5Stack_ATOM.h"
CHSV color = CHSV(0, 255, 255 * 2 / 3);
#endif

#include "OTAWebUpdater.h"


void setup() {
  //setCpuFrequencyMhz(160);

  Serial.begin(115200/*500E3 doesn't werk*/);
  while (!Serial);
  uint64_t efuseMac=ESP.getEfuseMac();
  if(efuseMac==0x7C3EAC7EB994) {
    ATOMCANBusKit=true;
#if not defined(CANSender_Debug)
    tx = true;
#endif
  }

#if defined(CANSender_BLE)
  setup_BLE();
#elif defined(BLE_server_multiconnect_NimBLE)
  setup_BLE_server_multiconnect_NimBLE();
#endif
#if defined(ARDUINO_TBeam) || defined(ARDUINO_M5Stack_ATOM)
#ifdef ARDUINO_TBeam
  setup_TTBEAM();
#endif
#ifdef ARDUINO_M5Stack_ATOM
  setup_M5Atom();
#endif
#else
  CAN.setPins(rx, tx); // TODO define
  int[] ids={0x101, 0x111, 0x2aa, 0x048, 0x0d2}
  int idsAND=0;
  for(int i=0; i<sizeof(ids); i++) idsAND &= ids[i];
  CAN.filter(idsAND, ~idsAND);
#endif

#ifdef CANSender_Debug
  Serial.print("efuseMac="); Serial.println(efuseMac, HEX);
  uint32_t Freq = 0;
  Freq = getCpuFrequencyMhz();
  Serial.print("CpuFrequency: "); Serial.print(Freq); Serial.println(" MHz");
  Freq = getXtalFrequencyMhz();
  Serial.print("XtalFrequency: "); Serial.print(Freq); Serial.println(" MHz");
  Freq = getApbFrequency();
  Serial.print("ApbFrequency: "); Serial.print(Freq); Serial.println(" Hz");
  /*Serial.println(ESP.setCpuFrequencyMhz(80));
  Serial.print("CpuFrequency:"); Serial.println(ESP.getCpuFreqMHz());*/

  Serial.println("CAN Sender");
#endif

  // start the CAN bus at 500 kbps
  if (!CAN.begin(500E3)) {
    Serial.println("Starting CAN failed!");
    while (1);
  }

  // register the receive callback
  CAN.onReceive(onReceive);
  Serial.println("% % | °C °C | W W (= V V x A A) Nm Nm rpm rpm ms ms motorControl W W");
}


unsigned int lowVoltage = 0;

unsigned int startByteMotor;
unsigned char checksumMotor, sumMotor;

unsigned int startByteLCD;
unsigned char checksumLCD, sumLCD;

unsigned long previousMillis = 0;
int interval = 200;

unsigned long previousMillisBLE = 0;
#ifdef CANSender_BLE
constexpr size_t intervalBLE = 2 * 1000;
#else
constexpr size_t intervalBLE =/*500*/1000 / 8;
#endif

#if defined(BLE_server_multiconnect_NimBLE)
unsigned long previousMillisBLE_server_multiconnect_NimBLE = 0;
constexpr size_t intervalBLE_server_multiconnect_NimBLE = 1 * 1000;
#endif

bool ota = false;
void loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    if (tx) {
      // send packet: id is 11 bits, packet can contain up to 8 bytes of data
      CAN.beginPacket(0x09A);
      CAN.write(0);
      CAN.endPacket();
#ifdef ARDUINO_M5Stack_ATOM
      color.setHSV(map(0x09A, 0, 0x2aa, 0, 255), 255, 255 * 2 / 3);
#endif
    } else {
#ifdef ARDUINO_M5Stack_ATOM
      color.setHSV(0, 0, 0);
#endif
    }
  }

  unsigned long currentMillisBLE = millis();
  if (currentMillisBLE - previousMillisBLE >= intervalBLE) {
    previousMillisBLE = currentMillisBLE;
#ifdef CANSender_BLE
    char zk[31 - 2];
    snprintf(zk, sizeof(zk), "%.1fV %.1fA %.1fW %.1fC %i%%", (float)voltage / 1000.0f + .05f, (float)current / 1000.0f + .05f, (float)power / 10.0f + .05f, (float)(temperature - 27315) / 100.0f + .05f, SoC);
    loop_BLE(zk);
#else
    char zk[128 - 2];
    //    snprintf(zk, sizeof(zk), "%i %% | %i mA | %i mW | %i mV | %i mK", SoC, current, power*100, voltage, temperature*10);
    //    snprintf(zk, sizeof(zk), "%i %% | %.2f \u2103 | %.1f W | %.2f V | %.3f A", SoC, (float)(temperature-27315)/100.0f+.005f, (float)power/10.0f+.05f, (float)voltage/1000.0f+.005f, (float)current/1000.0f+.0005f);
    // %02x
    //    snprintf(zk, sizeof(zk), "%i %% | %.2f \u2103 | %.1f W (= %.2f V × %.3f A) %i %i %i %i", SoC, (float)(temperature-27315)/100.0f, (float)power/10.0f, (float)voltage/1000.0f, (float)current/1000.0f, torqueTara, torqueActual, speedSensor, motorControl);
    snprintf(zk, sizeof(zk), "%i %% | %.2f C | %.1f W (= %.2f V x %.3f A) %i Nm %i rpm %i ms %i %i W", batteryLevel, (float)(temperature - 27315) / 100.0f, (float)power / 10.0f, (float)voltage / 1000.0f, (float)current / 1000.0f, torque/*Tara, torqueActual*/, cadence, speedSensor, motorControl, powerBio);

#if defined(BLE_server_multiconnect_NimBLE)
    unsigned long currentMillisBLE_server_multiconnect_NimBLE = millis();
    if (currentMillisBLE_server_multiconnect_NimBLE - previousMillisBLE_server_multiconnect_NimBLE >= intervalBLE_server_multiconnect_NimBLE) {
      previousMillisBLE_server_multiconnect_NimBLE = currentMillisBLE_server_multiconnect_NimBLE;
      loop_BLE_server_multiconnect_NimBLE();
    }
#endif
#endif
    Serial.println(zk);
  }

  unsigned char inByte;
#ifdef ARDUINO_M5Stack_ATOM
if(ATOMCANBusKit) {
  if (Serial2.available()) {
    // Tongsheng motor
    inByte = Serial2.read();
    if (inByte == 0x43) {
      unsigned char bufMotor[9 - (1)]; // -(inByte)
      Serial2.readBytes(bufMotor, sizeof(bufMotor));
      checksumMotor = bufMotor[9 - (1 + 1)]; // -(inByte+0based)
      sumMotor = inByte;
      for (int i = 1; i <= sizeof(bufMotor) - 1; i++) sumMotor += bufMotor[i - 1];
      if (checksumMotor == sumMotor) {
        batteryLevel = bufMotor[2 - (1 + 1)];
        motorStatus = bufMotor[3 - (1 + 1)];
        torque/*Tara*/ = bufMotor[4 - (1 + 1)];
        torqueSum += torque;
        torqueCount++;
        /*torqueActual*/cadence = bufMotor[5 - (1 + 1)];
#if defined(BLE_server_multiconnect_NimBLE)
        update_new_rpm(cadence);
#endif
        errorCode = bufMotor[6 - (1 + 1)];
        speedSensor = bufMotor[7 - (1 + 1)] + 256 * bufMotor[8 - (1 + 1)];
        startByteMotor++;
        color.setHSV(inByte, 255, 255 * 2 / 3);
      }
    }
  }

  // ToDo why while instead of if?
  while (Serial1.available()) {
    // Tongsheng LCD
    inByte = Serial1.read();
    if (inByte == 0x59) {
      unsigned char bufLCD[7 - 1];
      Serial1.readBytes(bufLCD, sizeof(bufLCD));
      checksumLCD = bufLCD[7 - (1 + 1)];
      sumLCD = inByte;
      for (int i = 1; i <= sizeof(bufLCD) - 1; i++) sumLCD += bufLCD[i - 1];
      if (checksumLCD == sumLCD) {
        motorControl = bufLCD[2 - (1 + 1)];
        startByteLCD++;
        color.setHSV(inByte, 255, 255 * 2 / 3);
      }
    }
  }
}
#endif

#ifdef CANSender_Debug
  torque = random(78, 200);
  cadence = random(90);
  batteryLevel = random(100);
#if defined(BLE_server_multiconnect_NimBLE)
  update_new_rpm(cadence);
#endif
#endif

#ifdef ARDUINO_TBeam
  loop_TTBEAM();
#endif
#ifdef ARDUINO_M5Stack_ATOM
  M5.update();
  if (M5.Btn.wasPressed()) {
#ifdef CANSender_Debug
    Serial.println("Pressed");
#endif
    //tx=!tx;
    tx=false;
    CAN.end();
    setup_OTAWebUpdater();
    ota = true;
  }
  /*color=!color;
    if(color && tx) M5.dis.drawpix(0, CHSV(0x09A,255,255));
    else M5.dis.clear();*/
  M5.dis.drawpix(0, color);
  if (ota) loop_OTAWebUpdater();
#endif
}

// IRAM_ATTR ? 
void onReceive(int packetSize) {
  // Bosch
  uint8_t buffer[8];
  CAN.readBytes(buffer, packetSize);
  int packetId=CAN.packetId();
#ifdef ARDUINO_M5Stack_ATOM
//  if (CAN.packetId() != 0x0E1 && CAN.packetId() != 0x0f1) color.setHSV(map(CAN.packetId(), 0, 0x2aa, 0, 255), 255, 255 * 2 / 3);
#endif
  switch (packetId) {
    /*case 0x09A: // DU present
      tx = false;
#ifdef ARDUINO_M5Stack_ATOM
  color.setHSV(map(packetId, 0, 0x2aa, 0, 255), 255, 255 * 2 / 3);
#endif
      break;*/
    case 0x101: // 500 ms
      //current = 256 * buffer[2] + buffer[3];
      bytes2int.myBytes[1] = buffer[2];
      bytes2int.myBytes[0] = buffer[3];
      current = bytes2int.myInt;
      power = 256 * buffer[4] + buffer[5];
      voltage = 256 * buffer[6] + buffer[7];
      /*if(voltage<30*1000) {
        lowVoltage++;
        color.setHSV(0,255,255*2/3);
        if(lowVoltage>100) {
          tx=false;
          lowVoltage=0;
        }
        } else {
        lowVoltage=0;
        }*/
      //      color=CHSV(0x101-2,255,255/2);
#ifdef ARDUINO_M5Stack_ATOM
  color.setHSV(map(packetId, 0, 0x2aa, 0, 255), 255, 255 * 2 / 3);
#endif
      break;
    case 0x111: // 100 ms
      batteryLevel = buffer[6];
      if (batteryLevel < 5) {
        lowVoltage++;
#ifdef ARDUINO_M5Stack_ATOM
        color.setHSV(0, 255, 255 * 2 / 3);
#endif
        if (lowVoltage > 100) {
          tx = false;
          lowVoltage = 0;
        }
      } else {
        lowVoltage = 0;
      }
      //      color=CHSV((uint8_t)CAN.packetId(),255,255/2);
#ifdef ARDUINO_M5Stack_ATOM
  color.setHSV(map(packetId, 0, 0x2aa, 0, 255), 255, 255 * 2 / 3);
#endif
      break;
    case 0x2aa: // 500 ms
      temperature = 256 * buffer[0] + buffer[1];
      //      color=CHSV((uint8_t)CAN.packetId(),255,255/2);
#ifdef ARDUINO_M5Stack_ATOM
  color.setHSV(map(packetId, 0, 0x2aa, 0, 255), 255, 255 * 2 / 3);
#endif
      break;
    /*case 0x0f1: // 10 ms
      //DoD=buffer[4];
      break;
    case 0x08C: // 200 ms
      break;
    case 0x01c: // 1000 ms
      break;
    case 0x0E1:
      break;
    case 0x151:
      break;*/
    case 0x048: // 10 ms
      //torque = (256 * buffer[0] + buffer[1])/10;
      bytes2int.myBytes[1] = buffer[0];
      bytes2int.myBytes[0] = buffer[1];
      torque = bytes2int.myInt;
      torqueSum += torque;
      torqueCount++;
      // too fast for blinkenlights
      break;
    case 0x0d2: // 100 ms
      cadence = buffer[1];
#if defined(BLE_server_multiconnect_NimBLE)
      // moved because LoadProhibited and Stack smashing protect failure
      //update_new_rpm(cadence);
#endif
#ifdef ARDUINO_M5Stack_ATOM
  color.setHSV(map(packetId, 0, 0x2aa, 0, 255), 255, 255 * 2 / 3);
#endif
      break;
#ifdef CANSender_Debug
    default:
      Serial.print(" 0x"); Serial.print(packetId, HEX); Serial.print(" ");
      break;
#endif
  }
}
