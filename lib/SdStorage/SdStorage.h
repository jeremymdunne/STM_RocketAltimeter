#ifndef _SD_STORAGE_H_
#define _SD_STORAGE_H_
#include <Arduino.h>
#include "SD.h"
#define SD_NO_FILES_AVAILABLE -1
#define SD_BEGIN_FAILED -2
#define SD_BUFFER_SIZE 500
#define SD_FILE_OPEN_FAILED -3


class SdStorage{
public:
  int begin();
  int writeData(String data);
  //void forceWrite();
  void close();
private:
  File flightData;
  void writeBuffer();
  int getFileName(String *name);

  int buffIndex = 0;
  char writeBuff[SD_BUFFER_SIZE] = {};
};

#endif
