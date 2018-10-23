#include <Arduino.h>
#include <BMP280.h>
#include <CircularFloatBuffer.h>
#include <SdStorage.h>

// Datasheet: https://cdn-shop.adafruit.com/datasheets/BST-BMP280-DS001-11.pdf

SdStorage fileStorage;
BMP280 bmp;
CircularFloatBuffer *recentAltitudeBuffer;
float baselinePressure = 0;
int ledPin = PC13;

float determineBaselinePressure(){
  //try to smooth out the pressure altitude readings
  //first get some bogus numbers, try to 'wake up' the sensor
  for(int i = 0; i < 5; i ++){
    bmp.getTemperature();
    bmp.getPressure();
    delay(10);
  }
  //now start running tally of pressure
  //assume no significant changes in temperature
  float pressureBaseline = bmp.getPressure();
  for(int i = 0; i < 50; i ++){
    pressureBaseline += bmp.getPressure();
    pressureBaseline /= 2.0;
    delay(10);
  }
  //return results
  return pressureBaseline;
}

float filterStore[5] = {};
int numDataInFilter = 0;
float filter(float newData){
    //simple filter for now
    //weighted average of past 5 values
    //test the circular buffer real quick
    recentAltitudeBuffer->add(newData);
    if(numDataInFilter < 5){
      numDataInFilter ++;
      return -1;
    }
    //otherwise, filter!
    recentAltitudeBuffer->getAll(&filterStore[0]);
    float running = 0;
    int divideVal = 0;
    for(int i = 0; i < 5; i ++){
      running += (i+1)*filterStore[i];
      divideVal += (i+1);
    }
    running/=divideVal;
    return running;
}



void setup() {
    // put your setup code here, to run once:
    recentAltitudeBuffer = new CircularFloatBuffer(5);
    pinMode(ledPin, OUTPUT);
    Serial.begin(9600);
    delay(2000);
    if(bmp.begin() == 0){
      Serial.println("BMP Connected");
    }
    else{
      Serial.println("BMP Failed to connect!");
      while(true);
    }
    int fileStorageStatus = fileStorage.begin();
    if(fileStorageStatus == 0){
      Serial.println("SD Connected");
    }
    else{
      if(fileStorageStatus == SD_NO_FILES_AVAILABLE){
        Serial.println("No Files Available!");
      }
      else{
        Serial.println("Failed SD, Check connections");
      }
      while(true);
    }

    bmp.setFilter(BMP280::Filter_16);
    delay(200);
    baselinePressure = 101325;
    digitalWrite(ledPin,HIGH);
    //filter(0);
}

void loop() {
    // put your main code here, to run repeatedly:
    long start = micros();
    float newAltitude = bmp.getAltitudeFromBaselinePressure(baselinePressure);
    float filteredAlt = filter(newAltitude);
    fileStorage.writeData("A"+String(newAltitude)+";" + String(millis())+";");
    //fileStorage.writeData();
    Serial.println(micros() - start);
    Serial.println("Raw Altitude: " + String(newAltitude));
    Serial.println("Filtered Altitude: " + String(filteredAlt));
    delay(1000/80.0);


}
