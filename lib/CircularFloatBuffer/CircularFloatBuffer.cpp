#include "CircularFloatBuffer.h"


CircularFloatBuffer::CircularFloatBuffer(int size){
  this->size = size;
  buffer = new float[this->size];
  curIndex = 0;
}

void CircularFloatBuffer::add(float value){
  buffer[curIndex] = value;
  curIndex ++;
  if(curIndex >= size){
    curIndex = 0;
  }
}

float CircularFloatBuffer::get(int index){
  int actualIndex = (curIndex + index) % size;
  return buffer[actualIndex];
}

void CircularFloatBuffer::getAll(float *buff){
  //copy the data into the buff
  for(int i = 0; i < size; i ++){
    buff[i] = get(i);
  }
}
