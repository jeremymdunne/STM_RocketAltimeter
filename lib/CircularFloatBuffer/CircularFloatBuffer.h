#ifndef _CIRCULAR_FLOAT_BUFFER_H_
#define _CIRCULAR_FLOAT_BUFFER_H_

class CircularFloatBuffer{
public:
  CircularFloatBuffer(int size);
  void add(float value);
  float get(int index);
  void getAll(float *buff);
private:
  float * buffer;
  int curIndex; //index of next member to fill
  int size;
};

#endif
