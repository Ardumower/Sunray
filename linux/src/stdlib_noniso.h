/* 
  stdlib_noniso.h - nonstandard (but usefull) conversion functions
*/

#ifndef STDLIB_NONISO_H
#define STDLIB_NONISO_H

#ifdef __cplusplus
extern "C"{
#endif

char* itoa (int val, char *s, int radix);

char* ltoa (long val, char *s, int radix);

char* utoa (unsigned int val, char *s, int radix);

char* ultoa (unsigned long val, char *s, int radix);
 
char* dtostrf (double val, signed char width, unsigned char prec, char *s);

void reverse(char* begin, char* end);

#ifdef __cplusplus
} // extern "C"
#endif


#endif

