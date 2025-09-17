/*
  Printable.h - Interface class that allows printing of complex types
*/

#ifndef Printable_h
#define Printable_h

#include <stdlib.h>

class Print;

/** The Printable class provides a way for new classes to allow themselves to be printed.
    By deriving from Printable and implementing the printTo method, it will then be possible
    for users to print out instances of this class by passing them into the usual
    Print::print and Print::println methods.
*/

class Printable
{
  public:
    virtual size_t printTo(Print& p) const = 0;
};

#endif

