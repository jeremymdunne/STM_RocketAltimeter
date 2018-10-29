#include <Arduino.h>
#include <BMP280.h>
#include <CircularFloatBuffer.h>
#include <SdStorage.h>
#include <MPU9250_IMU.h>
MPU9250_IMU::MPU_IMU_DATA data;
MPU9250_IMU imu;

// Datasheet: https://cdn-shop.adafruit.com/datasheets/BST-BMP280-DS001-11.pdf

SdStorage fileStorage;
BMP280 bmp;
CircularFloatBuffer *recentAltitudeBuffer;
float baselinePressure = 0;
int ledPin = PA0;

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
    Serial.begin(115200);
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
    if(imu.begin(200, 0) != 0){
      Serial.println("Init failed!");
      while(true);
    }
    bmp.setFilter(BMP280::Filter_16);
    delay(200);
    baselinePressure = 101325;
    digitalWrite(ledPin,HIGH);
    //filter(0);
}
long lastTime = 0;
float linearAcceleration = 0;
float runningAcceleration = 0;
void loop() {
    // put your main code here, to run repeatedly:
    long start = micros();
    imu.getData(&data);
    linearAcceleration = pow(pow(data.accel[0],2) + pow(data.accel[1],2) + pow(data.accel[2],2),0.5);
    runningAcceleration += linearAcceleration;
    runningAcceleration/=2.0;
    if(millis()-lastTime > 1000/20.0){
      float newAltitude = bmp.getAltitudeFromBaselinePressure(baselinePressure);
      //float filteredAlt = filter(newAltitude);
      fileStorage.writeData("A"+String(newAltitude)+";" + String(millis())+";");
      //fileStorage.writeData();
      Serial.println(micros() - start);
      Serial.println("Raw Altitude: " + String(newAltitude));
      lastTime = millis();
      Serial.println("Acceleration: " + String(runningAcceleration,4));
      runningAcceleration = linearAcceleration;
      fileStorage.writeData("G"+String(runningAcceleration,4)+";"+String(millis())+";");
    }


    //Serial.println("Filtered Altitude: " + String(filteredAlt));
    delay(2);


}
