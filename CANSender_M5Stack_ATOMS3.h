#include <M5Unified.h>
// for CHSV
#include <FastLED.h>


// ToDo: yAdvance
const int FONT_HEIGHT=0;

bool wasPressed() {
//  return M5.Btn.wasPressed();
  return M5.BtnA.wasReleased();
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


void setup_M5AtomS3() {
  Serial.setTxTimeoutMs(/*5*/50); // set USB CDC Time TX

  auto cfg = M5.config();
  M5.begin(cfg);

  M5.Display.setBrightness(255);
  M5.Lcd.setTextFont(4); // 1/2..4, 3 not visible (?)
if(ATOMCANBusKit) {
#ifdef TWAI
RX_PIN=gpio_num_t(6);
TX_PIN=gpio_num_t(5);
#else //TWAI
  CAN.setPins(GPIO_NUM_6, GPIO_NUM_5);
#endif //TWAI
#ifdef TONGSHENG
  Serial2.begin(9600, SERIAL_8N1, /*1*/GPIO_NUM_1);
  Serial1.begin(9600, SERIAL_8N1, /*2*/GPIO_NUM_2);
#endif
} else {
#ifdef TWAI
RX_PIN=GPIO_NUM_1;
TX_PIN=GPIO_NUM_2;
#else
  CAN.setPins(GPIO_NUM_1, GPIO_NUM_2);
#endif
}
}


long previousMillisM5AtomS3=0;
long intervalM5AtomS3=1000/10;
enum SCREEN {SCREEN_BATTERY, SCREEN_BIO, SCREEN_TX, SCREEN_OTA, ENUM_LAST};
SCREEN currentScreen;
void loop_M5AtomS3() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillisM5AtomS3 >= intervalM5AtomS3) {
    previousMillisM5AtomS3 = currentMillis;
    
    M5.Lcd.setCursor(0, 0);
    switch(currentScreen) {
      case SCREEN_BATTERY:
        M5.Lcd.setTextColor(GREEN,BLACK/*,true*/); M5.Lcd.printf("V | %.2f\n", (float)voltage/1000.0f);
        M5.Lcd.setTextColor(CYAN,BLACK/*,true*/); M5.Lcd.printf("A | %.3f\n", (float)current/1000.0f);
        M5.Lcd.setTextColor(RGB_24to16(255, 127, 255),BLACK/*,true*/); M5.Lcd.printf("W | %.1f\n", (float)power/10.0f);
        M5.Lcd.setTextColor(YELLOW,BLACK/*,true*/); M5.Lcd.printf("C | %.2f\n", (float)(temperature-27315)/100.0f);
        M5.Lcd.setTextColor(WHITE,BLACK/*,true*/); M5.Lcd.printf("%% | %i\n", batteryLevel);
        break;
      case SCREEN_BIO:
        M5.Lcd.setTextColor(RGB_24to16(255, 127, 255),BLACK/*,true*/); M5.Lcd.printf("W (el) %i\n", power/10);
        M5.Lcd.setTextColor(RGB_24to16(255, 127, 127),BLACK/*,true*/); M5.Lcd.printf("W (bio) %i\n", powerBio);
        M5.Lcd.setTextColor(YELLOW,BLACK/*,true*/); M5.Lcd.printf("W %i\n", power/10+powerBio);
        M5.Lcd.setTextColor(WHITE,BLACK/*,true*/); M5.Lcd.printf("rpm %i\n", cadence);
        break;
      case SCREEN_TX:
        M5.Lcd.setTextColor(WHITE,BLACK/*,true*/); M5.Lcd.printf("Tx: %i\n", tx);
        break;
      case SCREEN_OTA:
        M5.Lcd.setTextColor(WHITE,BLACK/*,true*/); M5.Lcd.printf("OTA\n", tx);
        break;
    }
  }
}
