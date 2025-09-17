/* 
 cbuf.h - Circular buffer implementation
 */

#ifndef __cbuf_h
#define __cbuf_h

#include <stddef.h>
#include <stdint.h>
#include <string.h>

class cbuf {
    public:
        cbuf(size_t size);
        ~cbuf();
        size_t getSize() const;
        size_t room() const ;
        bool empty() const ;
        bool full() const ;
        int peek() ;
        int read();
        size_t read(char* dst, size_t size);
        size_t write(char c);
        size_t write(const char* src, size_t size);
        void flush();

    private:
        const size_t _size;
        char* _buf;
        const char* const _bufend;
        char* _begin;
        char* _end;

        char* wrap_if_bufend(char* ptr) const ;
};

#endif//__cbuf_h
