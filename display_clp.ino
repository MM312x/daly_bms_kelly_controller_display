//I2C Adresses: Clock 0x68, Display 0x38, temp sensor: 0x48

#include <Wire.h>
#include <vector>
#include <string>
#include <unordered_map>
#include <Temperature_LM75_Derived.h>
#include <HardwareSerial.h>
#include "BLEDevice.h"

#define LGFX_USE_V1
#include <LovyanGFX.hpp>
#include <lvgl.h>
#include "src/ui.h"
#include "display.h"

#include "daly_bms.h"
#include "BT_connection.h"
#include "kelly_controller.h" 
#include "time_module.h"
#include "sd_card_module.h"
#include "errorMsg.h"


//Define GPIO
#define blinkerLeft 12
#define blinkerRight 35
#define highBeam 34
#define batteryCooling 5
#define brightnessSensor 32

//*********** Kelly setup ************
HardwareSerial KellySerial(1); //besser software serial?
Kelly kelly;

//*********** BMS setup************
bool dataUpdated = true;
BMS BMS;

ErrorManager errorManager;
unsigned long currentTime;
Generic_LM75 tempSensorPCB(0x48);

//BT callback when msg received
static void notifyCallback(
  BLERemoteCharacteristic* pBLERemoteCharacteristic,
  uint8_t* pData,
  size_t length,
  bool isNotify) {
    Serial.println("Notify callback for characteristic ");
    BMS.updateBmsData(pData, length);
    dataUpdated = true;
}


void updateDisplayBMS() {
  int current = BMS.getCurrent();
  if (current >= 0) {
    lv_slider_set_value(ui_powerSlider, current, LV_ANIM_ON);
    lv_slider_set_value(ui_powerRegenSlider, 0, LV_ANIM_ON);
  } else {
    lv_slider_set_value(ui_powerRegenSlider, current, LV_ANIM_ON);
    lv_slider_set_value(ui_powerSlider, 0, LV_ANIM_ON);
  }
  lv_label_set_text_fmt(ui_VoltageLabel, "%.1f V", BMS.getVoltage());
  lv_label_set_text_fmt(ui_AmpereLabel, "%.1f A", BMS.getCurrent());
  lv_label_set_text_fmt(ui_WattLabel, "%d W", BMS.getPower());
  lv_label_set_text_fmt(ui_batteryTempLabel, "%d / %d °C", BMS.getMinBatteryTemp(), BMS.getMaxBatteryTemp());
  lv_label_set_text_fmt(ui_cellMaxLabel, "%.3f V", BMS.getMaxCellVoltage()); 
  lv_label_set_text_fmt(ui_cellMinLabel, "%.3f V", BMS.getMinCellVoltage()); 
  lv_label_set_text_fmt(ui_cellDiffLabel, "%.3f V", BMS.getCellDiffVoltage()); 
  lv_label_set_text_fmt(ui_batteryPercentLabel, "%d %%", BMS.getBatteryPercent());
  lv_slider_set_value(ui_batteryFillLevelSlider, BMS.getBatteryPercent(), LV_ANIM_ON);
  lv_label_set_text_fmt(ui_ecoLabel, "%.2f Wh", BMS.getWattHour());
  lv_label_set_text_fmt(ui_averageAhLabel, "%.2f Wh/Km", BMS.getWatthPerKm(kelly.getDayKm()));
  lv_label_set_text_fmt(ui_rangeLabel, "%d Km", BMS.getRange()); 
}

void updateDisplayKelly() {
  lv_label_set_text_fmt(ui_SpeedLabel, "%.0f", kelly.getSpeed());
  lv_label_set_text_fmt(ui_motorControllerTempLabel, "%d / %d °C", kelly.getMotorTemp(), kelly.getControllerTemp());
  lv_label_set_text_fmt(ui_dayKmLabel, "%.1f Km", kelly.getDayKm());
  lv_label_set_text_fmt(ui_totalKmLabel, "%d Km", kelly.getTotalKm());
  //lv_label_set_text_fmt(ui_speedModeLabel, "%d ", kelly.get???());
}

void updateDisplayGPIO() {
  if (digitalRead(blinkerLeft)) {
    lv_obj_add_flag(ui_greenArrowLeftPNG, LV_OBJ_FLAG_HIDDEN);
  } else {
    lv_obj_clear_flag(ui_greenArrowLeftPNG, LV_OBJ_FLAG_HIDDEN);
  }
  if (digitalRead(blinkerRight)) {
    lv_obj_add_flag(ui_greenArrowRightPNG, LV_OBJ_FLAG_HIDDEN);
  } else {
    lv_obj_clear_flag(ui_greenArrowRightPNG, LV_OBJ_FLAG_HIDDEN);
  }
  /* anzeige display fehlt
  if (digitalRead(highBeam)) {
    lv_obj_add_flag(xxx, LV_OBJ_FLAG_HIDDEN);
  } else {
    lv_obj_clear_flag(xxx, LV_OBJ_FLAG_HIDDEN);
  }
  */
  int brightnessValue = analogRead(brightnessSensor); //ADC 12bit max 4095 dark: 1500, middle 2600, bright 4000
  Serial.print("LDR Sensor WErt: ");
  Serial.println(brightnessValue);
  if (brightnessValue > 3300) {
    tft.setBrightness(250);
  } else if (brightnessValue > 2000) {
    tft.setBrightness(180);
  } else {
    tft.setBrightness(50);
  }
}

void switchBatteryCooling() {
  if (BMS.getMaxBatteryTemp() > 22) {
    digitalWrite(batteryCooling, HIGH);
  } else {
    digitalWrite(batteryCooling, LOW);
  }
}

void updateDisplayTime() {
  byte second, minute, hour;
  readDS3231time(&second, &minute, &hour);
  if (minute < 10) { 
    lv_label_set_text_fmt(ui_TimeLabel, "%d:%d%d", hour, 0, minute);
  } else {
    lv_label_set_text_fmt(ui_TimeLabel, "%d:%d", hour, minute);
  }
}

void updateDisplayTemp() {
  lv_label_set_text_fmt(ui_outdoorTempLabel, "%.1f °C", tempSensorPCB.readTemperatureC());
}


void setup() {
  Wire.begin(18, 19);
  Serial.begin(115200);

  //setup GPIO
  pinMode(blinkerLeft, INPUT); //external pullup, no internal pullup
  pinMode(blinkerRight, INPUT_PULLUP); //internal pullup, strapping pin: no external pullup possible
  pinMode(highBeam, INPUT); //external pullup, no internal pullup
  pinMode(batteryCooling, OUTPUT);
  pinMode(brightnessSensor, INPUT);

  //display init
  initDisplay();
  lv_init();
  lv_disp_draw_buf_init(&draw_buf, buf, NULL, screenWidth * 10);
  /*Initialize the display*/
  static lv_disp_drv_t disp_drv;
  lv_disp_drv_init(&disp_drv);
  /*Change the following line to your display resolution*/
  disp_drv.hor_res = screenWidth;
  disp_drv.ver_res = screenHeight;
  disp_drv.flush_cb = my_disp_flush;
  disp_drv.draw_buf = &draw_buf;
  lv_disp_drv_register(&disp_drv);
  /*Initialize the (dummy) input device driver*/
  static lv_indev_drv_t indev_drv;
  lv_indev_drv_init(&indev_drv);
  indev_drv.type = LV_INDEV_TYPE_POINTER;
  indev_drv.read_cb = my_touchpad_read;
  lv_indev_drv_register(&indev_drv);
  ui_init();

  //SD Card
  if (initSDCard()) {
    int tempValue = round(tempSensorPCB.readTemperatureC());
    byte second, minute, hour, dayOfWeek, dayOfMonth, month, year;
    readDS3231TimeDate(&second, &minute, &hour, &dayOfWeek, &dayOfMonth, &month, &year);
    createFilename(readTotalKmFromSD(), dayOfMonth, month, year, hour, minute, tempValue);
    kelly.setTotalKmOld(readTotalKmFromSD());
  } else {
    kelly.setTotalKmOld(0);
    errorManager.reportError("SD Card Mount Failed --> no SD Card", millis());
  }
  
  // BLE init
  BLEDevice::init(""); 
  delay(200);

  // Kelly Serial init
  KellySerial.begin(19200, SERIAL_8N1, 4, 2); //rx 16, tx 17 green to RX2 and blue to TX2 black to GND (blau auf pin tx 2 und gruen auf pin rx 4)
  while (!Serial) {
    delay(10);
  }
  delay(100);
  KellySerial.write(kelly.command[0], 3);
  updateDisplayTemp();

  //***************************Set Time RTC********************
  // DS3231 seconds, minutes, hours, day (1=sunday), date, month, year
  //setDS3231time(30,25,12,3,17,12,24);
  //**************************************************************
}


void loop() {
  // display handler
  lv_timer_handler();
  currentTime = millis();
  
  //Bluetooth connect and request data from BMS
  if (btConnected) {
    if (dataUpdated) {
      BMS.clearOldErrorMsg();
      //BMS.printCurrentBmsData();
      dataUpdated = false;
      if (BMS.updateVoltCur) {
        pRemoteCharacteristic_write->writeValue(BMS.command[0], sizeof(BMS.command[0]));
        BMS.updateVoltCur = false;
        //Serial.println("Request BMS Data: Volt current SOC"); 
      } else {
        pRemoteCharacteristic_write->writeValue(BMS.command[BMS.msgData], sizeof(BMS.command[BMS.msgData]));
        BMS.msgData++;
        BMS.updateVoltCur = true;
        if (BMS.msgData > 4) {
          BMS.msgData = 1;
        }
        //Serial.print("Request BMS Data: "); 
        //Serial.println(BMS.msgData);
      }
      BMS.updateAmpWattHour();
      updateDisplayBMS();
      BMS.checkVoltage();
      if (BMS.errorActive()) {
        errorManager.reportError(BMS.getErrorMsg(), currentTime);
      }
    }
  } else {
    if (connectToServer()) {
      btConnected = true;
      Serial.println("We are now connected to the BLE Server.");
    } else {
      errorManager.reportError("BT failed to connected to BMS --> try to connect", currentTime);
      btConnected = false;
      resetDisplayBMS();
    }
  }

  // receive data from kelly controller
  if (KellySerial.available()) {
    kelly.timeout = 0;
    kelly.clearOldErrorMsg();
    std::vector<uint8_t> receivedData(64, 0);
    //Serial.print("Kelly received msg: ");

    //-------- NEU -----------
    int rvcdDataIdx = 0;
    long receiveTimeout = millis();
    //Serial.print("received data length: ");
    //Serial.println(KellySerial.available());
    while (KellySerial.available()) {
      receivedData.at(rvcdDataIdx) = KellySerial.read();
      rvcdDataIdx++;
      //Serial.print("rvcdDataIdx: ");
      //Serial.print(rvcdDataIdx);
      //Serial.print(": ");
      //Serial.println(receivedData.at(rvcdDataIdx-1));
      if ((millis() - receiveTimeout) > 200) {
        errorManager.reportError("Kelly receive timeout: no kelly data updated", currentTime);
        break;
      }      
    }
    std::vector<uint8_t> receivedDataShort(19, 0);
    //Serial.print("message length: ");
    //Serial.println(rvcdDataIdx);
    int firstByteIdx = 0;
    for (int i = 0; i < receivedData.size(); i++) {
      if (receivedData.at(i) == 58 || receivedData.at(i) == 59) {
        firstByteIdx = i;
        break;
      } 
    }
    for (int i = 0; i <= 18; i++) {
      receivedDataShort.at(i) = receivedData.at(i + firstByteIdx); 
      //Serial.print("received data sortiert: ");
      //Serial.print(i);
      //Serial.print(": ");
      //Serial.println(receivedDataShort.at(i));
    }
    kelly.updateKellyData(receivedDataShort);
    //-------- NEU ENDE-----------

    //kelly.printCurrentControllerData();
    KellySerial.write(kelly.command[kelly.commandNum], 3); //sizeof() instead of 3? --> sizeof(kelly.command[kelly.commandNum])
    kelly.commandNum++;
    if (kelly.commandNum > 1) {
      kelly.commandNum = 0;
    }
    if (kelly.getSpeed() >= 1 ) {
      writeTotalKmToSD(kelly.getTotalKm());
    }
    updateDisplayKelly();
    kelly.updateKm();
    if (kelly.errorActive()) {
      errorManager.reportError(kelly.getErrorMsg(), currentTime);
    }
  } else {
    kelly.timeout++;
    if (kelly.timeout > 10) {
      errorManager.reportError("Kelly connection timeout: send new request", currentTime);
      KellySerial.write(kelly.command[kelly.commandNum], 3);
      resetDisplayKelly();
    }
  }

  //GPIO read and update display
  updateDisplayGPIO();
  updateDisplayTime();
  //updateDisplayTemp(); //temp steigt im gehause stark an, nicht updaten da nur start wert richtige aussentemp liefert
  
  //Log data to SD card
  if (KellySerial.available() && btConnected) {
    writeToSD(BMS.getVoltage(), BMS.getCurrent(), BMS.getMaxBatteryTemp(), kelly.getSpeed(), kelly.getDayKm());
    switchBatteryCooling();
  }

  //show error msg on display
  if (errorManager.checkErrorActive()) {
    lv_textarea_set_text(ui_errorTextArea, errorManager.getErrorMsgs().c_str());
    lv_obj_clear_flag(ui_errorPNG, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(ui_errorTextArea, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(ui_ESimsonMackText, LV_OBJ_FLAG_HIDDEN);
    errorManager.clearExpiredErrors(millis());
  } else {
    lv_obj_add_flag(ui_errorPNG, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(ui_errorTextArea, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(ui_ESimsonMackText, LV_OBJ_FLAG_HIDDEN);
  }
  
  Serial.println("----------------------");
  delay(100);
}





