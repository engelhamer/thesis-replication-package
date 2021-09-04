/*
   SD card attached to SPI bus as follows:
 ** MOSI - pin 11
 ** MISO - pin 12
 ** CLK - pin 13
 ** CS - pin 4 (for MKRZero SD: SDCARD_SS_PIN)
*/

#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <Adafruit_INA219.h>

Adafruit_INA219 ina219;
File myFile;
int ledPin = 10;
long timestamp = millis();
long nextTimestamp = millis();
bool LOGGER_ON = false;

void setup() {  
  // Init Serial
  Serial.begin(115200);
  while (!Serial) { delay(1000); }
  
  // Init ina219
  while (!ina219.begin()) {
    delay(1000);
  }

  // Init SD card
  while (!SD.begin(4)) {
    delay(1000);
  }

  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);
}

void loop() {
  timestamp = millis();
  if (Serial.available() > 0) {
    String request = Serial.readStringUntil('\n');
    if (request == "START") {
      SD.remove("data.txt");
      while (!myFile) {
        myFile = SD.open("data.txt", FILE_WRITE);
      }
      LOGGER_ON = true;
      digitalWrite(ledPin, HIGH);
      nextTimestamp = millis();
      Serial.println(String(millis()));
    } else if (request == "STOP") {
      myFile.close();
      LOGGER_ON = false;
      digitalWrite(ledPin, LOW);
      while (!myFile) {
        myFile = SD.open("data.txt");
      }
      // read from the file until there's nothing else in it:
      while (myFile.available()) {
        Serial.write(myFile.read());
      }
      Serial.println("END");
      myFile.close();
    }
  }

  if(LOGGER_ON && timestamp >= nextTimestamp) {   
//    float shuntvoltage = 0;
//    float busvoltage = 0;
//    float current_mA = 0;
//    float loadvoltage = 0;
    float power_mW = 0;

//    shuntvoltage = ina219.getShuntVoltage_mV();
//    busvoltage = ina219.getBusVoltage_V();
//    current_mA = ina219.getCurrent_mA();
    power_mW = ina219.getPower_mW();
//    loadvoltage = busvoltage + (shuntvoltage / 1000);
    
    String line = ( String(timestamp) + "," +
//                    String(shuntvoltage) + "," + 
//                    String(busvoltage) + "," + 
//                    String(current_mA) + "," +
//                    String(loadvoltage) + "," +
                    String(power_mW));
                    
    myFile.println(line);

    nextTimestamp += 5; // 5 ms delay -> 200hz
  }
}