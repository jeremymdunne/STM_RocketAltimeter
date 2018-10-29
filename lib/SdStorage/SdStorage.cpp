#include "SdStorage.h"

int SdStorage::begin(){
  if(!SD.begin(PA4)){
    return SD_BEGIN_FAILED;
  }
  //open the file
  String name;
  int fileStatus = getFileName(&name);
  if(fileStatus != 0) return fileStatus;
  flightData = SD.open(name,O_CREAT | O_WRITE);
  delay(10);
  //if(flightData){
  //  Serial.println("Opened!");
  //}
  if(!flightData){
    return SD_FILE_OPEN_FAILED;
  }
  return 0;
}

int SdStorage::getFileName(String *name){
  String base = "fdat";
  int counter = 0;
  while(counter < 100){
    if(!SD.exists(base + String(counter) + ".txt")){
      *name = base + String(counter) + ".txt";
      return 0;
    }
    else counter ++;
  }
  return SD_NO_FILES_AVAILABLE;
}

int SdStorage::writeData(String data){
  //long startM = micros();
  //Serial.println("Writing " + data);
  if(buffIndex + data.length() + 1 >= SD_BUFFER_SIZE | (true)){
  //if(true){
    //Serial.println("Wririntg to sd card");
    flightData.write(writeBuff,buffIndex);
    //Serial.println("Mid");
    flightData.flush();
    //Serial.println("clearing mem");
    for(int i = 0; i < buffIndex; i ++){
      writeBuff[i] = '\0';
    }
    //Serial.println("done!");
    buffIndex = 0;
  }
  for(uint i = 0; i < data.length(); i ++){
    writeBuff[buffIndex] = data.charAt(i);
    buffIndex ++;
  }
  //Serial.println("Storage logic: " + String(micros() - startM));
  return 0;
}

void SdStorage::close(){
  flightData.close();
}
