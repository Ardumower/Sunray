/*
  file I/O
*/

#ifndef __FILEIO_H__
#define __FILEIO_H__

#include <Process.h>
#include <dirent.h>

#define FILE_READ "r"
#define FILE_WRITE "r+"
#define FILE_APPEND "a"
#define FILE_CREATE "w+"

//#define O_WRITE  FILE_WRITE
//#define O_APPEND FILE_APPEND
//#define O_READ   FILE_READ
//#define O_CREAT  FILE_CREATE 

namespace BridgeLib {

class File : public Stream {
  private:
    FILE *_file;
    DIR  *_dir;
    char *_name;

  public:
    //File(BridgeClass &b = Bridge);    
    File();        
    //File(const char *_filename, const char * _mode, BridgeClass &b = Bridge);
    File(const char *_filename, const char * _mode);    
    virtual ~File();

    virtual size_t write(uint8_t);
    virtual size_t write(const char *buf); 
    virtual size_t write(const uint8_t *buf, size_t size);
    virtual int read();
    virtual int peek();
    virtual int available();
    virtual void flush();
    int read(void *buf, uint16_t nbyte);
    boolean seek(uint32_t pos);
    uint32_t position();
    uint32_t size();
    void close();
    operator bool();
    const char * name();
    boolean isDirectory();
    File openNextFile(const char * mode = FILE_READ);
    void rewindDirectory(void);

    //using Print::write;
};

class FileSystemClass {
  public:
    //FileSystemClass(BridgeClass &_b = Bridge) { }

    boolean begin();
    boolean begin(int selectPin);

    // Open the specified file/directory with the supplied mode (e.g. read or
    // write, etc). Returns a File object for interacting with the file.
    // Note that currently only one file can be open at a time.
    File open(const char *filename, const char * mode = FILE_READ);
    File open(const String filename, const char * mode = FILE_READ){ return open(filename.c_str()); }
    

    // Methods to determine if the requested file path exists.    
    boolean exists(const char *filepath);
    boolean exists(const String filepath){ return exists(filepath.c_str()); }

    // Create the requested directory hierarchy--if intermediate directories
    // do not exist they will be created.
    boolean mkdir(const char *filepath);

    // Delete the file.
    boolean remove(const char *filepath);
    boolean remove(const String &filepath) {
        return remove(filepath.c_str());
    }

    boolean rmdir(const char *filepath);

  private:
    friend class File;
};

extern FileSystemClass FileSystem;

};

// We enclose File and FileSystem classes in namespace BridgeLib to avoid
// conflicts with legacy SD library.

// This ensure compatibility with older sketches that uses only Bridge lib
// (the user can still use File instead of BridgeFile)
using namespace BridgeLib;

// This allows sketches to use BridgeLib::File together with SD library
// (you must use BridgeFile instead of File when needed to disambiguate)
typedef BridgeLib::File            BridgeFile;
typedef BridgeLib::FileSystemClass BridgeFileSystemClass;
#define BridgeFileSystem           BridgeLib::FileSystem

#define SD FileSystem

#endif
