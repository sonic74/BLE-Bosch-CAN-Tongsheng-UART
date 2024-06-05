#include "M5Atom.h"


bool wasPressed() {
  return M5.Btn.wasPressed();
}


void drawpix(int led, CHSV color) {
  M5.dis.drawpix(led, color);
}


void setup_M5Atom() {
  // void begin(bool SerialEnable = true, bool I2CEnable = true, bool DisplayEnable = false);
  M5.begin(true, false, true);
if(ATOMCANBusKit) {
  // ATOM CANBus Kit (CA-IS3050G) K057:
  // G19: RX
  // G22: TX
#ifdef TWAI
#define RX_PIN GPIO_NUM_19
#define TX_PIN GPIO_NUM_22
#else
  CAN.setPins(GPIO_NUM_19, GPIO_NUM_22);
#endif

  // https://wiki.seeedstudio.com/Grove_System/#grove-uart
  // https://github.com/hurzhurz/tsdz2/blob/master/pinout.md
  // 1k resistor?
  // Serial2.begin(unsigned long baud, uint32_t config, int8_t rxPin, int8_t txPin, bool invert)
  Serial2.begin(9600, SERIAL_8N1, 32 /*Grove RX yellow <- TSDZ2 TX brown */);
  Serial1.begin(9600, SERIAL_8N1, 26 /*Grove TX white  <- LCD TX / TSDZ2 RX orange */);
} else {
  // CANBus Unit(CA-IS3050G) U085:
  // G32: RX
  // G26: TX
#ifdef TWAI
#define RX_PIN GPIO_NUM_32
#define TX_PIN GPIO_NUM_26
#else
  CAN.setPins(GPIO_NUM_32, GPIO_NUM_26);
#endif
}
}
