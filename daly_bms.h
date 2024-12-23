class BMS {
  public:
    void updateBmsData(uint8_t* pReceivedData, size_t length) {
      if (length == 13) {
        for (int i = 0; i <= length - 1; i++) {
          //Serial.print(String(*(pReceivedData + i), HEX));
          //Serial.print(" ");
          receivedData_[i] = *(pReceivedData + i);
        }
        if (calcCheckSum(receivedData_)) {
          if (receivedData_[2] >= 144 && receivedData_[2] <= 152) {
            for (int i = 0; i<= 12; i++) {
              currentBmsData_[receivedData_[2]-144][i] = receivedData_[i];
              //Serial.print(receivedData_[i]);
            }
          } else {
            errorMsg_ = "BMS: Received data ID can not be decoded, data not updated";
            Serial.println("BMS: Received data ID can not be decoded, data not updated");
          }
        } else {
          errorMsg_ = "BMS: BT wrong checksum, data not updated";
          Serial.println("BMS: BT wrong checksum, data not updated");
        }
      } else {
        Serial.println("BMS: received data wrong length");
        errorMsg_ = "BMS: received data wrong length";
      }
      //Serial.print("updated array: ");
      //Serial.println(receivedData_[2]);
    }

    void printCurrentBmsData() {
      for (int i = 0; i <= 12; i++) {
        Serial.print(currentBmsData_[0][i]);
        Serial.print(" ");
      }
      Serial.println();
    }

    float getVoltage() {
      uint16_t iVoltage = 0;
      float voltage = 0.0;
      iVoltage <<= 8;
      iVoltage |= currentBmsData_[0][4];
      iVoltage <<= 8;
      iVoltage |= currentBmsData_[0][5];
      voltage = iVoltage/10.0;
      return voltage;
    }

    float getSoc() {
      uint16_t iStateOfCharge = 0;
      float stateOfCharge = 0.0;
      iStateOfCharge <<= 8;
      iStateOfCharge |= currentBmsData_[0][10];
      iStateOfCharge <<= 8;
      iStateOfCharge |= currentBmsData_[0][11];
      stateOfCharge = iStateOfCharge/10.0;
      return stateOfCharge;
    }

    float getCurrent() {
      uint16_t iCurrent = 0;
      float current = 0.0;
      iCurrent <<= 8;
      iCurrent |= currentBmsData_[0][8];
      iCurrent <<= 8;
      iCurrent |= currentBmsData_[0][9];
      current = (iCurrent-30000)/10.0;      
      return current;
    }

    int getPower() {
      int power = getCurrent() * getVoltage();     
      return power;
    }

    //alte berechnung ueber spannung
    /*
    int getBatteryPercent() {
      voltageBuffer_[voltageBufferIdx] = getVoltage();
      voltageBufferIdx++;
      if (voltageBufferIdx >= 10) {
        voltageBufferIdx = 0;
      }
      float bufferSum = 0;
      for (int i = 0; i<=9; i++) {
        bufferSum = bufferSum + voltageBuffer_[i];
      }
      int batteryPercent = round(((bufferSum / 10.0) - 42.0) * (100.0 / 12.2)); //full 54.2V empty 42V
      return batteryPercent;
    }
    */

    int getBatteryPercent() {
      if ((millis() - timeStartRange_) > 5000) { //in den ersten paar sekunden kommen falsche werte
        if (startCapacityWh_ < 1.0) {
          startCapacityWh_ = (getVoltage() - VOLTAGE_EMPTY_) * AVERAGE_Wh_PRO_VOLT_;       
          restCapacityWh_ = startCapacityWh_;
        } else {
          restCapacityWh_ = startCapacityWh_ - getWattHour();
        }
      } 
      int batteryPercent = round((100.0 / CAPACITY_FULL_BAT_) * restCapacityWh_);
      if (batteryPercent > 100) {
        batteryPercent = 100;
      }
      if (batteryPercent < 0) {
        batteryPercent = 0;
      }
      return batteryPercent;
    }

    void updateAmpWattHour() {
      int timeframe = millis() - timeAmpCalc_;
      timeAmpCalc_ = millis(); 
      Serial.print("loop time amp calc: ");
      Serial.println(timeframe);
      if (timeframe < 700) { //greater 700 --> unrealistic value
        ampMilliSec_ = round(ampMilliSec_ + (getCurrent() * timeframe));
        wattMilliSec_ = round(wattMilliSec_ + (getCurrent() * getVoltage() * timeframe));
        ampHour_ = ampMilliSec_ / 3600000.0; 
        wattHour_ = wattMilliSec_ / 3600000.0;
      } else {
        Serial.println("WARNING: loop takes too long");
      }
    }

    float getAmpHour() { //not in use 
      return ampHour_;
    }

    float getWattHour() {
      return wattHour_;
    }

    float getAmphPerKm(float km) { //not in use
      if (km > 0.01) {
        amphPerKm_ = ampHour_ / km;
      }
      return amphPerKm_;
    }

    float getWatthPerKm(float km) {
      if (km > 0.01) {
        watthPerKm_ = wattHour_ / km;
      }
      return watthPerKm_;
    }

    //alte berechnung
    /*
    int getRange() {
      int range = 0;
      if ((millis() - timeStartRange_) < 10000) { //in den ersten paar sekunden kommen falsche werte
        restCapacity_ = 30.0 - ((54.2 - getVoltage()) * 2.46); //annahme 30ah hat der volle akku, 2,46 ist ein schaetzwert
      } else {
        range = round((restCapacity_ - ampHour_) / amphPerKm_);
      }
      return range;
    }
    */

    int getRange() {
      int range = 0;
      range = round(restCapacityWh_ / watthPerKm_);
      return range;
    }

    void checkVoltage() {
      float volt = getVoltage();
      if (volt < 44.5 && volt > 42.0 ) { //assumption 3.5V drop while acceleration: speed 2 between 47.5V and 45V
        errorMsg_ = "Low Voltage: Switch to Speed 2";
      }
      if (volt <= 42.0 && volt > 37.0 ) { //assumption 3.5V drop while acceleration: speed 1 below 45V
        errorMsg_ = "Very Low Voltage: Switch to Speed 1";
      }
    }


    float getMaxCellVoltage() {
      uint16_t iMaxCellVoltage = 0;
      float maxCellVoltage = 0.0;
      iMaxCellVoltage <<= 8;
      iMaxCellVoltage |= currentBmsData_[1][4];
      iMaxCellVoltage <<= 8;
      iMaxCellVoltage |= currentBmsData_[1][5];
      maxCellVoltage = iMaxCellVoltage/1000.0;
      //int cellNumMaxVolt = currentBmsData_[1][6];
      return maxCellVoltage;
    }

    float getMinCellVoltage() {
      uint16_t iMinCellVoltage = 0;
      float minCellVoltage = 0.0;
      iMinCellVoltage <<= 8;
      iMinCellVoltage |= currentBmsData_[1][7];
      iMinCellVoltage <<= 8;
      iMinCellVoltage |= currentBmsData_[1][8];
      minCellVoltage = iMinCellVoltage/1000.0;
      //int cellNumMinVolt = currentBmsData_[1][9];
      return minCellVoltage;
    }

    float getCellDiffVoltage() {
      float cellDiffVoltage = getMaxCellVoltage() - getMinCellVoltage();
      return cellDiffVoltage;
    }

    int getMaxBatteryTemp() {
      int maxBatteryTemp = currentBmsData_[2][4]-40;
      //uint8_t sensorMaxTemp = receivedDataBuffer[2][5];
      return maxBatteryTemp;
    }

    int getMinBatteryTemp() {
      int minBatteryTemp = currentBmsData_[2][6]-40;
      //uint8_t sensorMinTemp = currentBmsData_[2][7];
      return minBatteryTemp;
    }

    bool errorActive() {
      for (int byteIdx = 0; byteIdx < 7; byteIdx++) {
        if (currentBmsData_[8][4+byteIdx] != 0) {
          return true;
        }
      }
      if (errorMsg_.length() > 4) {
        return true;
      }
      return false;
    }

    void clearOldErrorMsg() {
      errorMsg_ = "";
    }

    std::string getErrorMsg() {
      //uint8_t testData[13] = {0xa5, 0x80, 0x97, 0x08, 0x02, 0x00, 0x40, 0x00, 0xa4, 0x00, 0x00, 0x00, 0xc4} ;
      //errorMsg_ = "";
      for(int byteIdx = 0; byteIdx < 7; byteIdx++) {
        for(int bitIdx = 0; bitIdx < 8; bitIdx++) {
          int isBitSet = currentBmsData_[8][4+byteIdx] & (0x1 << bitIdx);
          //int isBitSet = testData[4+byteIdx] & (0x1 << bitIdx);
          if(isBitSet) {
              int idx = byteIdx*8+bitIdx;
              errorMsg_ += " & " + preDevErrorMsgs_[idx];
          }
        }
      }
      return errorMsg_;
    }

    uint8_t msgData = 0;
    bool updateVoltCur = true;
    uint8_t command[9][13] = {
      {0xa5, 0x80, 0x90, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xbd},  // 0 voltage, current, SOC
      {0xa5, 0x80, 0x91, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xbe},  // 1 max cell voltage, number of max cell voltage, min cell voltage, number of min cell voltage
      {0xa5, 0x80, 0x92, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xbf},  // 2 min max temp
      {0xa5, 0x80, 0x98, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc5},  // 8 error msg
      {0xa5, 0x80, 0x93, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc0},  // 3 charge discharge  MOS status
      {0xa5, 0x80, 0x94, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc1},  // 4 status info 1
      {0xa5, 0x80, 0x95, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc2},  // 5 cell voltage
      {0xa5, 0x80, 0x96, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc3},  // 6 temp
      {0xa5, 0x80, 0x97, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc4}   // 7 equilibrium state
    };

  private:
    bool calcCheckSum(uint8_t msg[13]) {
      int sum = 0;
      for (int i = 0; i < 12; i++) {
        sum += msg[i]; 
      }
      if (sum%256 == msg[12]) {
        return true;
      } else {
        return false;
      }
    }
    
    std::string preDevErrorMsgs_[64] = {
      //Byte 0:
      "one stage warning of unit over voltage", //0
      "one stage warning of unit over voltage", //1
      "one stage warning of unit over voltage", //2
      "two stage warning of unit over voltage", //3
      "Total voltage is too high One alarm", //4
      "Total voltage is too high Level two alarm", //5
      "Total voltage is too low One alarm", //6
      "Total voltage is too low Level two alarm", //7

      //Byte 1:
      "Charging temperature too high. One alarm", //8
      "Charging temperature too high. Level two alarm", //9
      "Charging temperature too low. One alarm", //10
      "Charging temperature's too low. Level two alarm", //11
      "Discharge temperature is too high. One alarm", //12
      "Discharge temperature is too high. Level two alarm", //13
      "Discharge temperature is too low. One alarm", //14
      "Discharge temperature is too low. Level two alarm", //15

      //Byte 2:
      "Charge over current. Level one alarm", //16
      "Charge over current, level two alarm", //17
      "Discharge over current. Level one alarm", //18
      "Discharge over current, level two alarm", //19
      "SOC is too high an alarm", //20
      "SOC is too high. Alarm Two", //21
      "SOC is too low. level one alarm", //22
      "SOC is too low. level two alarm", //23

      //Byte 3:
      "Excessive differential pressure level one alarm", //24
      "Excessive differential pressure level two alarm", //25
      "Excessive temperature difference level one alarm", //26
      "Excessive temperature difference level two alarm", //27 
      " - ", //28
      " - ", //29
      " - ", //30
      " - ", //31 

      //Byte 4:
      "Charging MOS overtemperature warning", //32
      "discharge MOS overtemperature warning", //33
      "charging MOS temperature detection sensor failure", //34
      "discharge MOS temperature detection sensor failure", //35
      "charging MOS adhesion failure", //36
      "discharge MOS adhesion failure", //37
      "charging MOS breaker failure", //38
      "discharge MOS breaker failure", //39

      //Byte 5:
      "AFE acquisition chip malfunction", //40
      "monomer collect drop off", //41
      "Single Temperature Sensor Fault", //42
      "EEPROM storage failures", //43
      "RTC clock malfunction", //44
      "Precharge Failure", //45
      "vehicle communications malfunction", //46
      "intranet communication module malfunction", //47

      //Byte 6:
      "Current Module Failure", //48
      "main pressure detection module", //49
      "Short circuit protection failure", //50
      "Low Voltage No Charging", //51
      " - ", //52
      " - ", //53
      " - ", //54
      " - " //55

      //Byte 7: ?
    };

    uint8_t receivedData_[13];
    uint8_t currentBmsData_[9][13];
    float voltageBuffer_[10] = {53.0, 53.0, 53.0, 53.0, 53.0, 53.0, 53.0, 53.0, 53.0, 53.0};
    int ampMilliSec = 0;
    float restCapacity_ = 0.0;
    float startCapacityWh_ = 0.0;
    float restCapacityWh_ = 0.0;
    float ampHour_ = 0.0;
    float wattHour_ = 0.0;
    float amphPerKm_ = 0.55;
    float watthPerKm_ = 26.5;
    uint8_t voltageBufferIdx = 0;
    int ampMilliSec_ = 0;
    int wattMilliSec_ = 0;
    std::string errorMsg_;
    unsigned long timeAmpCalc_ = millis();
    unsigned long timeStartRange_ = millis();
    static const int VOLTAGE_EMPTY_ = 44; //40V limit --> 4V voltage drop while acceleration
    static const int AVERAGE_Wh_PRO_VOLT_ = 135; //Wh/V average out of drive data
    static const int CAPACITY_FULL_BAT_ = 1296; //Wh 53.6V full battery average out of drive data -> 53.6-44=9.6 -> 9.6*135=12964Wh

};