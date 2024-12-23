#include "Print.h"
#include "esp32-hal-adc.h"
#include "WString.h"
#include "FS.h"
#include "stdio.h"
#include <SPI.h>
#include "SD.h"

#define _SD_SCLK  25
#define _SD_MISO  27
#define _SD_MOSI  26
#define _SD_CS    33

File myFile; 
String filename = "";
bool sdAvailable = false;
SPIClass SPI_SD(VSPI);

bool initSDCard() {
  SPI_SD.begin(_SD_SCLK, _SD_MISO, _SD_MOSI, _SD_CS);
  if (!SD.begin(_SD_CS, SPI_SD)) {
    sdAvailable = false;
    return(0);
  }
  sdAvailable = true;
  return(1);    
}

int readFile(fs::FS &fs, const char * path){
    //Serial.printf("Reading file: %s\n", path);
    File file = fs.open(path);
    if(!file){
        Serial.println("Failed to open file for reading");
        return 0;
    }
    int myNumber = 0;
    while (file.available()){
      char c = file.read();
      if(c == '\n'){
      } else {
          myNumber = myNumber * 10;
          myNumber = myNumber + (c - '0');  // Subtract '0' to adjust from ascii back to real numbers
      }
    }
    file.close();
    return myNumber;
}

void writeFile(fs::FS &fs, const char * path, const char * message){
    //Serial.printf("Writing file: %s\n", path);
    File file = fs.open(path, FILE_WRITE);
    if(!file){
        Serial.println("Failed to open file for writing");
        return;
    }
    if(!file.print(message)){
        Serial.println("Write failed");
    }
    file.close();
}

void appendFile(fs::FS &fs, const char * path, const char * message){
    //Serial.printf("Appending to file: %s\n", path);
    File file = fs.open(path, FILE_APPEND);
    if(!file){
        Serial.println("Failed to open file for appending");
        return;
    }
    if(!file.print(message)){
        Serial.println("Append failed");
    }
    file.close();
}

void writeToSD(float volt, float amp, float batteryTemp, int speed, float dayKm) {
  if (sdAvailable) {
    String dataMessage = String(millis()) + "," + String(volt) + "," + String(amp) + "," + String(batteryTemp) + ","  + String(speed) + "," + String(dayKm) + "\r\n";
    //Serial.print("data msg: ");
    //Serial.println(dataMessage.c_str());
    appendFile(SD, filename.c_str(), dataMessage.c_str());
  }
}

void writeTotalKmToSD(int totalKm) {
  if (sdAvailable) {
    String km = String(totalKm);
    writeFile(SD, "/totalKm.txt", km.c_str());
  }
}

int readTotalKmFromSD() {
  if (sdAvailable) {
    int km = readFile(SD, "/totalKm.txt");
    return km;
  }
  return 0;
}

void createFilename(int totalKm, byte dayOfMonth, byte month, byte year, byte hour, byte min, int tempValue) { 
  filename = "/" + String(totalKm) + "_" + String(dayOfMonth, DEC) + "_" + String(month) + "_" + String(year, DEC) + "_" + String(hour, DEC) + "_" + String(min, DEC) + "_" + String(tempValue) + ".txt" + "\r\n";
  //Serial.print("filename: ");
  //Serial.println(filename.c_str());
}






