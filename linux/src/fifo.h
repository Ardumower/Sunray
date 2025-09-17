#ifndef FIFO_H_
#define FIFO_H_


#include <Arduino.h>


// example usage:  
//   FiFo<string, 1000> myFifo;
//   FiFo<struct mystruct, 1000> myFifo;
//   FiFo<char, 1000> myFifo;

// TData: The data type of the fifo.
// len: The length of the fifo.
 
template<typename TData, uint16_t TLen>
class FiFo
{
  public:    
    bool available(){
      	return (fifoRead != fifoWrite); 
    }  
    
    bool read(TData &frame){
      if (!available ()){
        return false;
      }
  	  frame = fifo[fifoRead = (fifoRead + 1) % TLen];
      return true;
    }

    bool peek(TData &frame){
      if (!available ()){
        return false;
      }
  	  frame = fifo[(fifoRead + 1) % TLen];
      return true;
    }

    bool write(TData frame){
       uint16_t tmpWrite = (fifoWrite + 1) % TLen;
        if (tmpWrite != fifoRead)
        {
          fifo[tmpWrite] = frame;
          fifoWrite = tmpWrite;
          return true;
        } else {
          return false;
        }
    }

  protected:
    uint16_t fifoRead = 0;
    uint16_t fifoWrite = 0;
    TData fifo[TLen];
};


#endif /* FIFO_H */
