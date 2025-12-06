#if defined(ARDUINO_M5STACK_Core2)
#include <M5Core2.h>
#endif

// untested
#if defined(ARDUINO_M5Stick_C_PLUS)
#include <M5StickCPlus.h>
#endif

/*#include <M5GFX.h>
M5GFX display;*/
//#include "Display_Unicode/CUF_24px.h"
TFT_eSprite img = TFT_eSprite(&M5.Lcd);
// ToDo: yAdvance
const int FONT_HEIGHT=24;


bool wasPressed() {
#if defined(ARDUINO_M5Stack_ATOMS3)
  return M5.Btn.wasPressed();
#else
  if(M5.BtnA.wasPressed()) {
#if defined(ARDUINO_M5STACK_Core2)
    M5.Spk.DingDong();
    M5.Axp.SetLDOEnable(3, true);  //Open the vibration.   开启震动马达
    delay(100);
    M5.Axp.SetLDOEnable(3, false);
#endif
#if defined(ARDUINO_M5Stick_C_PLUS)
    M5.Beep.beep();
#endif
    return true;
  }
  return false;
#endif // defined(ARDUINO_M5Stack_ATOMS3)
}


int RGB_24to16(int red, int grn, int blu) {
  return ( ((red >> 3) << 11) | ((grn >> 2) << 5) | (blu >> 3) );
}

int32_t pointer=0;
void drawpix(int led, CHSV color) {
  CRGB colorRGB = CRGB(color);
//  M5.Lcd.drawFastHLine(0,M5.Lcd.height()-1,M5.Lcd.width()-1,RGB_24to16(colorRGB[0],colorRGB[1],colorRGB[2]));
//  M5.Lcd.drawPixel(pointer++%M5.Lcd.width(),M5.Lcd.height()-1,RGB_24to16(colorRGB[0],colorRGB[1],colorRGB[2]));
  M5.Lcd.drawFastVLine(pointer++%M5.Lcd.width(),M5.Lcd.height()-2,M5.Lcd.height()-1,RGB_24to16(colorRGB[0],colorRGB[1],colorRGB[2]));
}


void setup_M5Core2() {
#ifdef CANSender_Debug
  Serial.println("setup_M5Core2()");
  Serial.printf("Total heap: %d\n", ESP.getHeapSize());
  Serial.printf("Free heap: %d\n", ESP.getFreeHeap());
  Serial.printf("Total PSRAM: %d\n", ESP.getPsramSize());
  Serial.printf("Free PSRAM: %d\n", ESP.getFreePsram());
#endif
  M5.begin();
#if defined(ARDUINO_M5STACK_Core2)
  M5.Lcd.setFreeFont(&FreeSans18pt7b);
  img.setFreeFont(&FreeSans18pt7b);
#endif
  M5.Lcd.setCursor(0, M5.Lcd.height()-1);
  M5.Lcd.setTextColor(WHITE); M5.Lcd.print("OTA");
  img.setColorDepth(8);
  img.createSprite(M5.Lcd.width(), M5.Lcd.height()-FONT_HEIGHT);
  
  // CANBus Unit(CA-IS3050G) U085:
  // G33: RX
  // G32: TX
#ifdef TWAI
RX_PIN=GPIO_NUM_33
TX_PIN=GPIO_NUM_32
#else
  CAN.setPins(GPIO_NUM_33, GPIO_NUM_32);
#endif
}


long previousMillisM5Core2=0;
long intervalM5Core2=1000/10;
void loop_M5Core2() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillisM5Core2 >= intervalM5Core2) {
    previousMillisM5Core2 = currentMillis;
    
    float battVoltage, battDischargeCurrent, battPower, battTemp;
#if defined(ARDUINO_M5STACK_Core2)
    if(M5.Axp.isCharging()) {
      battVoltage=M5.Axp.GetVBusVoltage();
      battDischargeCurrent=M5.Axp.GetBatCurrent()*-1.0f;
    } else {
      battVoltage=M5.Axp.GetBatVoltage();
      battDischargeCurrent=M5.Axp.GetBatCurrent()*-1.0f;
    }
    battPower=M5.Axp.GetBatPower();
    battTemp=M5.Axp.GetTempInAXP192();
#endif
    
    img.setCursor(0, FONT_HEIGHT);
    //M5.Lcd.clear(BLACK);
    img.fillSprite(BLACK);
    img.setTextColor(GREEN  ); img.printf("V | %.2f | %.2f\n", (float)voltage/1000.0f, battVoltage);
    img.setTextColor(CYAN   ); img.printf("A | %.3f | %.3f\n", (float)current/1000.0f, battDischargeCurrent/1000.0f);
    img.setTextColor(MAGENTA); img.printf("W | %.1f    | %.1f\n", (float)power/10.0f, battPower/1000.0f);
    img.setTextColor(YELLOW ); img.printf("C | %.2f | %.2f\n", (float)(temperature-27315)/100.0f, battTemp);
    img.setTextColor(WHITE  ); img.printf("%% | %i\n", batteryLevel);
    img.pushSprite(0, 0/*, BLACK*/);
    // already in loop()
    //M5.update();  //Read the press state of the key.  读取按键 A, B, C 的状态
  }
}
