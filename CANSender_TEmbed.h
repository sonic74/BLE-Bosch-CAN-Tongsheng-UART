/*
Z:\SvensHDD\Download\Treiber\Espressif\esptool-v4.4-win64\esptool.exe --chip esp32s3 --port "COM12" --baud 921600  --before usb_reset --after hard_reset write_flash  -z --flash_mode dio --flash_freq 80m --flash_size 16MB 0x0 "C:\Users\sven\AppData\Local\Temp\arduino-sketch-4D8592AB2BFFC8E345FCA125F8968262/BLE-Bosch-CAN-Tongsheng-UART.ino.bootloader.bin" 0x8000 "C:\Users\sven\AppData\Local\Temp\arduino-sketch-4D8592AB2BFFC8E345FCA125F8968262/BLE-Bosch-CAN-Tongsheng-UART.ino.partitions.bin" 0xe000 "C:\Users\sven\AppData\Local\Arduino15\packages\esp32\hardware\esp32\2.0.5/tools/partitions/boot_app0.bin" 0x10000 "C:\Users\sven\AppData\Local\Temp\arduino-sketch-4D8592AB2BFFC8E345FCA125F8968262/BLE-Bosch-CAN-Tongsheng-UART.ino.bin"
"C:\Users\sven\AppData\Local\Arduino15\packages\esp32\tools\esptool_py\4.5.1/esptool.exe" --chip esp32s3 --port "COM12" --baud 921600  --before usb_reset --after hard_reset write_flash  -z --flash_mode dio --flash_freq 80m --flash_size 4MB 0x0 "C:\Users\sven\AppData\Local\Temp\arduino\sketches\4D8592AB2BFFC8E345FCA125F8968262/BLE-Bosch-CAN-Tongsheng-UART.ino.bootloader.bin" 0x8000 "C:\Users\sven\AppData\Local\Temp\arduino\sketches\4D8592AB2BFFC8E345FCA125F8968262/BLE-Bosch-CAN-Tongsheng-UART.ino.partitions.bin" 0xe000 "C:\Users\sven\AppData\Local\Arduino15\packages\esp32\hardware\esp32\2.0.15/tools/partitions/boot_app0.bin" 0x10000 "C:\Users\sven\AppData\Local\Temp\arduino\sketches\4D8592AB2BFFC8E345FCA125F8968262/BLE-Bosch-CAN-Tongsheng-UART.ino.bin" 
*/


#include <TFT_eSPI.h>       // https://github.com/Xinyuan-LilyGO/T-Embed/tree/main/lib/TFT_eSPI
// undefined reference
//#include "TEmbed/TFT_eSPI/TFT_eSPI.h"
// glcdfont.c:8:35: error: expected '=', ',', ';', 'asm' or '__attribute__' before 'PROGMEM'
//#include "src/TEmbed/TFT_eSPI/TFT_eSPI.h"
#include <RotaryEncoder.h>
#include <FastLED.h>
CHSV color = CHSV(0, 255, 255 * 2 / 3);
// unnecessary?
//#include "TEmbed/fonts.h"
TFT_eSPI tft = TFT_eSPI();
TFT_eSprite img = TFT_eSprite(&tft);

#define NUM_LEDS 7
#define DATA_PIN 42
#define CLOCK_PIN 45
CRGB leds[NUM_LEDS];

#define PIN_IN1 2
#define PIN_IN2 1



void setup_TEmbed() {
  pinMode(0, INPUT_PULLUP);
  pinMode(46, OUTPUT);
  digitalWrite(46, HIGH);

  pinMode(15, OUTPUT);
  digitalWrite(15, HIGH);

  FastLED.addLeds<APA102, DATA_PIN, CLOCK_PIN, RGB>(leds, NUM_LEDS);

  tft.begin();
  tft.writecommand(0x11);
  tft.setRotation(3);
  tft.setSwapBytes(true);
  tft.fillScreen(TFT_BLACK);
  tft.setTextDatum(4);

  img.setSwapBytes(true);
  img.createSprite(320, 170);


  leds[0] = CRGB::Red;
  leds[1] = CRGB::Green;
  leds[2] = CRGB::Blue;
  leds[3] = CRGB::Cyan;
  leds[4] = CRGB::Magenta;
  leds[5] = CRGB::Yellow;
  leds[6] = CRGB::White;
  FastLED.show();



// Pins used to connect to CAN bus transceiver:
#ifdef TWAI
#define RX_PIN 43
#define TX_PIN 44
#else
  CAN.setPins(43, 44);
#endif
}


void loop_TEmbed() {
  char zk[80];
//  img.drawString((float)voltage/1000.0f, 10, 10);
  img.fillSprite(TFT_BLACK);
  img.setTextColor(TFT_WHITE, TFT_BLACK);
  img.setFreeFont(&FreeSans12pt7b);
  sprintf(zk, "%.2f V", (float)voltage/1000.0f);
  img.drawString(zk, 10, 10);
  sprintf(zk, "%.3f A", (float)current/1000.0f);
  img.drawString(zk, 10, 40);
  sprintf(zk, "%.1f W", (float)power/10.0f);
  img.drawString(zk, 10, 70);
  sprintf(zk, "%.2f °C", (float)(temperature-27315)/100.0f);
  img.drawString(zk, 10, 100);
  sprintf(zk, "%i %%", batteryLevel);
  img.drawString(zk, 10, 130);
  img.pushSprite(0, 0);

  leds[0] = color;
  // "No hardware SPI pins defined.  All SPI access will default to bitbanged output"
  //FastLED.show();
}
