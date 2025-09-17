/*
  Print.h - Base class that provides print() and println()
*/

#ifndef Print_h
#define Print_h

#include <inttypes.h>
#include <stdio.h> // for size_t

#include "WString.h"
#include "Printable.h"

#define DEC 10
#define HEX 16
#define OCT 8
#define BIN 2

class Print{
  private:
    int write_error;
    size_t printNumber(unsigned long, uint8_t);
    size_t printFloat(double, uint8_t);
  protected:
    void setWriteError(int err = 1) { write_error = err; }
  public:
    Print(): write_error(0) {}
    virtual ~Print(){}

    int getWriteError() { return write_error; }
    void clearWriteError() { setWriteError(0); }
  
    virtual size_t write(const uint8_t c){ return 0; }
    virtual size_t write(const uint8_t *buffer, size_t size);
    size_t write(const char *buffer, size_t size) { return write((const uint8_t *)buffer, size); }
    size_t write(const char *str){
      if (str == NULL) return 0;
      return write((const uint8_t *)str, strlen(str));
    }
    
    size_t printf(const char *format, ...);
    size_t print(const String &);
    size_t print(const char*);
    size_t print(char);
    size_t print(unsigned char, int = DEC);
    size_t print(int, int = DEC);
    size_t print(unsigned int, int = DEC);
    size_t print(long, int = DEC);
    size_t print(unsigned long, int = DEC);
    size_t print(double, int = 2);
    size_t print(const Printable&);

    size_t println(const String &s);
    size_t println(const char*);
    size_t println(char);
    size_t println(unsigned char, int = DEC);
    size_t println(int, int = DEC);
    size_t println(unsigned int, int = DEC);
    size_t println(long, int = DEC);
    size_t println(unsigned long, int = DEC);
    size_t println(double, int = 2);
    size_t println(const Printable&);
    virtual size_t println(void);
};

#endif
