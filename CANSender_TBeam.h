// https://github.com/lewisxhe/AXP202X_Library
#include <axp20x.h>

AXP20X_Class PMU;

bool initPMU()
{
    if (PMU.begin(Wire, AXP192_SLAVE_ADDRESS) == AXP_FAIL) {
        return false;
    }
    /*
     * The charging indicator can be turned on or off
     * * * */
    // PMU.setChgLEDMode(LED_BLINK_4HZ);

    /*
    * The default ESP32 power supply has been turned on,
    * no need to set, please do not set it, if it is turned off,
    * it will not be able to program
    *
    *   PMU.setDCDC3Voltage(3300);
    *   PMU.setPowerOutPut(AXP192_DCDC3, AXP202_ON);
    *
    * * * */

    /*
     *   Turn off unused power sources to save power
     * **/

    PMU.setPowerOutPut(AXP192_DCDC1, AXP202_OFF);
    PMU.setPowerOutPut(AXP192_DCDC2, AXP202_OFF);
    PMU.setPowerOutPut(AXP192_LDO2, AXP202_OFF);
    PMU.setPowerOutPut(AXP192_LDO3, AXP202_OFF);
    PMU.setPowerOutPut(AXP192_EXTEN, AXP202_OFF);

    /*
     * Set the power of LoRa and GPS module to 3.3V
     **/
/*    PMU.setLDO2Voltage(3300);   //LoRa VDD
    PMU.setLDO3Voltage(3300);   //GPS  VDD*/
    PMU.setDCDC1Voltage(3300);  //3.3V Pin next to 21 and 22 is controlled by DCDC1

    PMU.setPowerOutPut(AXP192_DCDC1, AXP202_ON);
/*    PMU.setPowerOutPut(AXP192_LDO2, AXP202_ON);
    PMU.setPowerOutPut(AXP192_LDO3, AXP202_ON);
*/
#define PMU_IRQ                     35
    pinMode(PMU_IRQ, INPUT_PULLUP);
    attachInterrupt(PMU_IRQ, [] {
        // pmu_irq = true;
    }, FALLING);

    PMU.adc1Enable(AXP202_VBUS_VOL_ADC1 |
                   AXP202_VBUS_CUR_ADC1 |
                   AXP202_BATT_CUR_ADC1 |
                   AXP202_BATT_VOL_ADC1,
                   AXP202_ON);

    PMU.enableIRQ(AXP202_VBUS_REMOVED_IRQ |
                  AXP202_VBUS_CONNECT_IRQ |
                  AXP202_BATT_REMOVED_IRQ |
                  AXP202_BATT_CONNECT_IRQ,
                  AXP202_ON);
    PMU.clearIRQ();

    return true;
}

void initBoard()
{
#define I2C_SDA                     21
#define I2C_SCL                     22
    Wire.begin(I2C_SDA, I2C_SCL);

    if(!initPMU()) {
      Serial.println("initPMU failed!");
    }

#define BOARD_LED                   4
#define LED_ON                      LOW
#define LED_OFF                     HIGH
#ifdef BOARD_LED
    /*
    * T-BeamV1.0, V1.1 LED defaults to low level as trun on,
    * so it needs to be forced to pull up
    * * * * */
#if LED_ON == LOW
    gpio_hold_dis(GPIO_NUM_4);
#endif
    pinMode(BOARD_LED, OUTPUT);
    digitalWrite(BOARD_LED, LED_ON);
#endif
}


#include <SSD1306.h>
SSD1306         *oled = nullptr;

void setup_TTBEAM() {
  initBoard();

  oled = new SSD1306(0x3C, I2C_SDA, I2C_SCL);
  oled->init();
  oled->flipScreenVertically();
  oled->setFont(ArialMT_Plain_16);
  oled->setTextAlignment(TEXT_ALIGN_LEFT);
  oled->drawString(0, 0, "hello world");
  oled->display();

  // CRX, CTX
#ifdef TWAI
#define RX_PIN 4
#define TX_PIN 15
#else
  CAN.setPins(4, 15);
#endif
}

void loop_TTBEAM() {
  char zk[80];
  oled->clear();
  float battVoltage=PMU.getBattVoltage()/1000.0f;
  sprintf(zk, "%.2f | %.2f V", (float)voltage/1000.0f, battVoltage);
  oled->drawString(0, 0, zk);
  float battDischargeCurrent=PMU.getBattDischargeCurrent()/1000.0f;
  sprintf(zk, "%.3f | %.3f A", (float)current/1000.0f, battDischargeCurrent);
  oled->drawString(0, 16, zk);
  sprintf(zk, "%.1f | %.1f W", (float)power/10.0f, battVoltage*battDischargeCurrent);
  oled->drawString(0, 32, zk);
  sprintf(zk, "%.2f °C | %i %%", (float)(temperature-27315)/100.0f, batteryLevel);
  oled->drawString(0, 48, zk);
  oled->display();
}
