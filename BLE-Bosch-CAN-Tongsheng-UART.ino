/* Bosch CAN PowerPack activation and Bosch & Tongsheng TSDZ2 Garmin compatible Bluetooth LE Cycling Power Profile
 *
 * works with esp32 2.0.4
 * worked with M5Stack 2.0.3 & M5Atom 0.0.8 (esp32 1.0.6)
 *
 * Copyright 2021-2022 Sven Killig (sven@killig.de)
 * http://sven.killig.de/Bosch/CANSender/
 */

//bool ATOMCANBusKit=false;
bool ATOMCANBusKit=true;
#define CANSender_Debug
#define TWAI
//#define TONGSHENG
//bool tx = false;
bool tx = true;
// ################################################################################################################################

#ifdef TWAI
#include "driver/twai.h"
gpio_num_t RX_PIN=GPIO_NUM_NC, TX_PIN=GPIO_NUM_NC;
#else
// doesn't build for ARDUINO_M5STACK_ATOMS3
// https://github.com/sandeepmistry/arduino-CAN
#include <CAN.h>
#endif

int16_t current, power;
uint16_t voltage;
int temperature = 27315;

unsigned char batteryLevel, motorStatus, /*torqueTara, torqueActual*/ cadence, errorCode;
int16_t torque;
int torqueSum, torqueCount;
unsigned int speedSensor;

int16_t powerBio;

unsigned char motorControl;

// undefined behaviour
union bytes2int_t {
  uint8_t myBytes[2];
  int16_t myInt;
} bytes2int;
union bytes2uint_t {
  uint8_t myBytes[2];
  uint16_t myUint;
} bytes2uint;


//#define CANSender_BLE
#define BLE_server_multiconnect_NimBLE
#if defined(CANSender_BLE)
#include "CANSender_BLE.h"
#elif defined(BLE_server_multiconnect_NimBLE)
#include "BLE_server_multiconnect_NimBLE.h"
#endif

#ifdef ARDUINO_M5STACK_ATOM
#include "CANSender_M5Stack_ATOM.h"
#endif

#if defined(ARDUINO_M5STACK_Core2) || defined(ARDUINO_M5Stick_C_PLUS)
// for CHSV
#include <FastLED.h>
#include "CANSender_M5Stack_Core2.h"
#endif

#if defined(ARDUINO_M5STACK_ATOMS3)
// blocks if not read out
//#define Serial USBSerial
#include "CANSender_M5Stack_ATOMS3.h"
#endif

#if defined(ARDUINO_M5STACK_ATOM) || defined(ARDUINO_M5STACK_Core2) || defined(ARDUINO_M5Stick_C_PLUS) || defined(ARDUINO_M5STACK_ATOMS3)
#define ARDUINO_M5Stack
CHSV color = CHSV(0, 255, 255 * 2 / 3);
#endif

#if defined(ARDUINO_TBeam) || defined(ARDUINO_TBEAM_USE_RADIO_SX1262)
#include "CANSender_TBeam.h"
#endif

#ifdef ARDUINO_ESP32S3_DEV
#define ARDUINO_TEmbed
// USB\VID_303A&PID_1001&MI_00\8&2BFFE741&0&0000
#include "CANSender_TEmbed.h"
#endif

#include "OTAWebUpdater.h"


void setup() {
  //setCpuFrequencyMhz(160);

  Serial.begin(115200/*500E3 doesn't werk*/);
  while (!Serial);
  uint64_t efuseMac=ESP.getEfuseMac();
  if(efuseMac==0x7C3EAC7EB994 /*|| efuseMac==0x2C7CC77554DC*/) {
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
#if defined(ARDUINO_TBeam) || defined(ARDUINO_TBEAM_USE_RADIO_SX1262) || defined(ARDUINO_TEmbed) || defined(ARDUINO_M5Stack)
#if defined(ARDUINO_TBeam) || defined(ARDUINO_TBEAM_USE_RADIO_SX1262)
  setup_TTBEAM();
#endif
#ifdef ARDUINO_TEmbed
  setup_TEmbed();
#endif
#ifdef /*ARDUINO_M5Stack_ATOM*/ARDUINO_M5STACK_ATOM
  setup_M5Atom();
#endif
#if defined(ARDUINO_M5STACK_Core2) || defined(ARDUINO_M5Stick_C_PLUS)
  setup_M5Core2();
#endif
#if defined(/*ARDUINO_M5Stack_ATOMS3*//*ARDUINO_m5stack_atoms3*/ARDUINO_M5STACK_ATOMS3)
  setup_M5AtomS3();
#endif
#else // defined(ARDUINO_TBeam) || defined(ARDUINO_TEmbed) || defined(ARDUINO_M5Stack)
  CAN.setPins(rx, tx); // TODO define
/*  int[] ids={0x101, 0x111, 0x2aa, 0x048, 0x0d2}
  int idsAND=0;
  for(int i=0; i<sizeof(ids); i++) idsAND &= ids[i]; // TODO?
#ifdef CANSender_Debug
  Serial.print("idsAND=0b"); Serial.print(idsAND, BIN); 
#endif
  CAN.filter(idsAND, ~idsAND);*/
#endif // defined(ARDUINO_TBeam) || defined(ARDUINO_TEmbed) || defined(ARDUINO_M5Stack)

#ifdef CANSender_Debug
  Serial.printf("ATOMCANBusKit=%i\n", ATOMCANBusKit);
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
#ifdef TWAI
  Serial.printf("TX_PIN=%i, RX_PIN=%i\n", TX_PIN, RX_PIN);
  // Initialize configuration structures using macro initializers
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(TX_PIN, RX_PIN, TWAI_MODE_NORMAL/*TWAI_MODE_NO_ACK*/);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL()/*{.acceptance_code = (0x111 << 21),.acceptance_mask = ~(0x7FF << 21),.single_filter = true}*/;

  // Install TWAI driver
  ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
  Serial.println("Driver installed");

  // Start TWAI driver
  ESP_ERROR_CHECK(twai_start());
  Serial.println("Driver started");
  gpio_dump_io_configuration(stdout, (1ULL << TX_PIN) | (1ULL << RX_PIN));

#ifdef CANSender_Debug
  // Reconfigure alerts to detect frame receive, Bus-Off error and RX queue full states
  uint32_t alerts_to_enable = /*TWAI_ALERT_RX_DATA | TWAI_ALERT_ERR_PASS | TWAI_ALERT_BUS_ERROR | TWAI_ALERT_RX_QUEUE_FULL | TWAI_ALERT_BUS_OFF | TWAI_ALERT_TX_FAILED*/TWAI_ALERT_ALL;
#else
  uint32_t alerts_to_enable = TWAI_ALERT_RX_DATA;
#endif
  ESP_ERROR_CHECK(twai_reconfigure_alerts(alerts_to_enable, NULL));
  Serial.println("CAN Alerts reconfigured");
#else
  if (!CAN.begin(500E3)) {
    Serial.println("Starting CAN failed!");
    while (1);
  }

  // register the receive callback
  CAN.onReceive(onReceive);
#endif
  Serial.println("% % | °C °C | W W (= V V x A A) Nm Nm rpm rpm ms ms motorControl W W");
}


unsigned int lowVoltage = 0;

unsigned int startByteMotor;
unsigned char checksumMotor, sumMotor;

unsigned int startByteLCD;
unsigned char checksumLCD, sumLCD;

int interval = 200;
unsigned long previousMillis = 0;
bool tx_cycle;

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
uint32_t lastChangeFor;
void loop() {
#ifdef TWAI
  // Check if alert happened
  uint32_t alerts_triggered;
  /*ESP_ERROR_CHECK(*/twai_read_alerts(&alerts_triggered, 1)/*)*/;
  twai_status_info_t twaistatus;
  ESP_ERROR_CHECK(twai_get_status_info(&twaistatus));

  // Handle alerts
  if (alerts_triggered) {
/*    Serial.printf("Alert: ");*/
  }
  if (alerts_triggered & TWAI_ALERT_ERR_PASS) {
    Serial.println("Alert: Alert: Alert: Alert: Alert: Alert: Alert: Alert: Alert: Alert: Alert: Alert: Alert: Alert: Alert: Alert: Alert: Alert: Alert: TWAI controller has become error passive.");
  }
  if (alerts_triggered & TWAI_ALERT_BUS_OFF) {
    Serial.println("Alert: Alert: Alert: Alert: Alert: Alert: Alert: Alert: Alert: Alert: Alert: Alert: Alert: Alert: Alert: Alert: Alert: Alert: Alert: TWAI_ALERT_BUS_OFF");
  }
  if (alerts_triggered & TWAI_ALERT_TX_FAILED) {
    Serial.println("Alert: Alert: Alert: Alert: Alert: Alert: Alert: Alert: Alert: Alert: Alert: Alert: Alert: Alert: Alert: Alert: Alert: Alert: Alert: TWAI_ALERT_TX_FAILED");
  }
  if (alerts_triggered & TWAI_ALERT_BUS_ERROR) {
    Serial.println("Alert: Alert: Alert: Alert: Alert: Alert: Alert: Alert: Alert: Alert: Alert: Alert: Alert: Alert: Alert: Alert: A (Bit, Stuff, CRC, Form, ACK) error has occurred on the bus.");
    Serial.printf("Bus error count: %d\n", twaistatus.bus_error_count);
  }
  if (alerts_triggered & TWAI_ALERT_RX_QUEUE_FULL) {
/*    Serial.println("Alert: Alert: Alert: Alert: Alert: Alert: Alert: Alert: Alert: Alert: Alert: Alert: Alert: Alert: Alert: Alert: Alert: The RX queue is full causing a received frame to be lost.");
    Serial.printf("RX buffered: %d\t", twaistatus.msgs_to_rx);
    Serial.printf("RX missed: %d\t", twaistatus.rx_missed_count);
    Serial.printf("RX overrun %d\n", twaistatus.rx_overrun_count);*/
  }

  // Check if message is received
  if (alerts_triggered & TWAI_ALERT_RX_DATA) {
    // One or more messages received. Handle all.
    onReceive(-1);
  }
#endif

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    if (tx) {
      // send packet: id is 11 bits, packet can contain up to 8 bytes of data
#ifdef TWAI
      /*leds[1] = CRGB::Black;
      FastLED.show();*/
#ifdef CANSender_Debug
      switch (twaistatus.state) {
        case TWAI_STATE_STOPPED: Serial.printf("\tTWAI_STATE_STOPPED\t"); break;
        case TWAI_STATE_RUNNING: /*Serial.printf("\tTWAI_STATE_RUNNING\t");*/ break;
        case TWAI_STATE_BUS_OFF: Serial.printf("\tTWAI_STATE_BUS_OFF\t"); break;
        case TWAI_STATE_RECOVERING: Serial.printf("\tTWAI_STATE_RECOVERING\t"); break;
        default: Serial.printf("\tTWAI_STATE unknown!\t"); break;
      }
#endif
      //ESP_ERROR_CHECK(twai_clear_transmit_queue());
      //Configure message to transmit
      twai_message_t message;
      message.identifier = 0x09A;
      message.extd = 0;
      message.data_length_code = 1;
      message.data[0]=0;
      //message.ss = 1;		//<<<Special mode - Single shot - TX will be attempted only once
      esp_err_t esp_err;
      esp_err=twai_transmit(&message, pdMS_TO_TICKS(interval));
      switch (esp_err) {
        case  ESP_OK:
//        ESP_ERROR_CHECK(twai_transmit(&message, pdMS_TO_TICKS(1000)));
#ifdef ARDUINO_TEmbed
          leds[1] = tx_cycle ? CRGB::Red : CRGB::Blue;
          tx_cycle=!tx_cycle;
          FastLED.show();
#endif
          break;
        case ESP_ERR_INVALID_ARG: Serial.println("Error: Error: Error: Error: Error: Error: Error: Error: Error: Error: Error: Error: Error: Error: Error: Error: Error: Error: ESP_ERR_INVALID_ARG!"); break;
        case ESP_ERR_TIMEOUT: Serial.println("Error: Error: Error: Error: Error: Error: Error: Error: Error: Error: Error: Error: Error: Error: Error: Error: Error: Error: ESP_ERR_TIMEOUT!"); break;
        case ESP_FAIL: Serial.println("Error: Error: Error: Error: Error: Error: Error: Error: Error: Error: Error: Error: Error: Error: Error: Error: Error: Error: ESP_FAIL!"); break;
        case ESP_ERR_INVALID_STATE: Serial.println("Error: Error: Error: Error: Error: Error: Error: Error: Error: Error: Error: Error: Error: Error: Error: Error: Error: Error: ESP_ERR_INVALID_STATE!"); break;
        case ESP_ERR_NOT_SUPPORTED: Serial.println("Error: Error: Error: Error: Error: Error: Error: Error: Error: Error: Error: Error: Error: Error: Error: Error: Error: Error: ESP_ERR_NOT_SUPPORTED!"); break;
        default: Serial.printf("Error: Error: Error: Error: Error: Error: Error: Error: Error: Error: Error: Error: Error: Error: Error: Error: Error: Error: %i!\n", esp_err); break;
/*        ESP_ERROR_CHECK(twai_stop());
        ESP_ERROR_CHECK(twai_start());*/
      }
#else//def TWAI
      CAN.beginPacket(0x09A);
      CAN.write(0);
      CAN.endPacket();
#ifdef CANSender_Debug
      Serial.println("CAN.endPacket()");
#endif
#endif
#ifdef ARDUINO_M5Stack
      color.setHSV(map(0x09A, 0, 0x2aa, 0, 255), 255, 255 * 2 / 3);
    } else {
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
//#ifndef ARDUINO_TEmbed
    Serial.println(zk);
//#endif
  }

#ifdef TONGSHENG
if(ATOMCANBusKit) {
  unsigned char inByte;
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
#endif // ARDUINO_M5Stack

#ifdef CANSender_Debug
  /*torque*/powerBio = random(78, 200);
  cadence = random(1, 90);
  batteryLevel = random(100);
#if defined(BLE_server_multiconnect_NimBLE)
  update_new_rpm(cadence);
#endif
#endif

#if defined(ARDUINO_TBeam) || defined(ARDUINO_TBEAM_USE_RADIO_SX1262)
  loop_TTBEAM();
#endif
#ifdef ARDUINO_TEmbed
  loop_TEmbed();
#endif
#if defined(ARDUINO_M5STACK_Core2) || defined(ARDUINO_M5Stick_C_PLUS)
  loop_M5Core2();
#endif
#if defined(ARDUINO_M5STACK_ATOMS3)
  loop_M5AtomS3();
#endif
#if defined(ARDUINO_M5STACK_Core2) || defined(ARDUINO_M5Stick_C_PLUS) || defined(ARDUINO_M5STACK_ATOMS3)
  // circular dependency!
  lastChangeFor=millis()-M5.BtnA.lastChange();
  M5.update();
  if (wasPressed()) {
#ifdef CANSender_Debug
    Serial.println("Pressed");
#endif
    if(lastChangeFor>1500) {
/*      tx=!tx;*/
      switch(currentScreen) {
        case SCREEN_TX:
          tx=!tx;
          break;
        case SCREEN_OTA:
          setup_OTAWebUpdater();
          ota = true;
          break;
      }
    } else {
/*      tx=false;
#ifdef TWAI
      twai_stop();
#else
      CAN.end();
#endif
      setup_OTAWebUpdater();
      ota = true;*/
      M5.Lcd.clear(BLACK);
      currentScreen=static_cast<SCREEN>((currentScreen+1)%(ENUM_LAST));
    }
  }
  /*color=!color;
    if(color && tx) M5.dis.drawpix(0, CHSV(0x09A,255,255));
    else M5.dis.clear();*/
  drawpix(0, color);
  if (ota) loop_OTAWebUpdater();
#endif // defined(ARDUINO_M5STACK_Core2) || defined(ARDUINO_M5Stick_C_PLUS) || defined(ARDUINO_M5STACK_ATOMS3)
}


// IRAM_ATTR ? 
void onReceive(int packetSize) {
  // Bosch
  uint8_t buffer[8];
  int packetId;
#ifdef TWAI
  twai_message_t message;
while/*if*/ (twai_receive(&message, /*0*/pdMS_TO_TICKS(1)) == ESP_OK) {
    if(message.extd || message.rtr) {
#ifdef CANSender_Debug
      Serial.println("message.extd || message.rtr");
#endif
      return;
    }
    packetId=message.identifier;
    packetSize=message.data_length_code;
    //*buffer=*(message.data);
    //for(int i=0; i<packetSize; i++) buffer[i]=message.data[i];
    memcpy(buffer, message.data, packetSize);

    /*if(packetId==0x101 || packetId==0x111 || packetId==0x2aa) {
    Serial.print("packetId=0x");
    Serial.println(packetId, HEX);
    Serial.print("packetSize=");
    Serial.println(packetSize);
    Serial.print("message.data[");
    for(int i=0; i<packetSize; i++) {
      Serial.print(i);
      Serial.print("]=0x");
      Serial.print(message.data[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
    Serial.print("buffer[");
    for(int i=0; i<packetSize; i++) {
      Serial.print(i);
      Serial.print("]=0x");
      Serial.print(buffer[i], HEX);
      Serial.print(" ");
      buffer[i]=message.data[i];
    }
    Serial.println();
    }*/
#else
  CAN.readBytes(buffer, packetSize);
  packetId=CAN.packetId();
#endif
#ifdef ARDUINO_M5Stack
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
      bytes2uint.myBytes[1] = buffer[6];
      bytes2uint.myBytes[0] = buffer[7];
      //voltage = 256 * buffer[6] + buffer[7];
      voltage = bytes2uint.myUint;
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
#ifdef ARDUINO_M5Stack
  color.setHSV(map(packetId, 0, 0x2aa, 0, 255), 255, 255 * 2 / 3);
#endif
      break;
    case 0x111: // 100 ms
      batteryLevel = buffer[6];
      if (batteryLevel < 5) {
        lowVoltage++;
#ifdef ARDUINO_M5Stack
        color.setHSV(0, 255, 255 * 2 / 3);
#endif
        if (lowVoltage > 100*10) {
#ifdef CANSender_Debug
          Serial.println("lowVoltage > 1000 !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
#endif
#ifdef ARDUINO_M5STACK_Core2
          M5.Spk.DingDong();
#endif
          tx = false;
          lowVoltage = 0;
        }
      } else {
        lowVoltage = 0;
      }
      //      color=CHSV((uint8_t)CAN.packetId(),255,255/2);
#ifdef ARDUINO_M5Stack
  color.setHSV(map(packetId, 0, 0x2aa, 0, 255), 255, 255 * 2 / 3);
#endif
      break;
    case 0x2aa: // 500 ms
      temperature = 256 * buffer[0] + buffer[1];
      //      color=CHSV((uint8_t)CAN.packetId(),255,255/2);
#ifdef ARDUINO_M5Stack
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
#ifdef ARDUINO_M5Stack
  color.setHSV(map(packetId, 0, 0x2aa, 0, 255), 255, 255 * 2 / 3);
#endif
      break;
#ifdef CANSender_Debug
/*    default:
      Serial.print(" 0x"); Serial.print(packetId, HEX); Serial.print(" ");
      break;*/
#endif
  }
#ifdef TWAI
}
#endif
}
