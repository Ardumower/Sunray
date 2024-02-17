/*
  Copyright (c) 2013 Arduino LLC. All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA
*/

#include <FileIO.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <dirent.h>
#include <Console.h>

namespace BridgeLib {

//File::File(BridgeClass &b):_file(NULL),_dir(NULL),_name(NULL) {}

File::File():_file(NULL),_dir(NULL),_name(NULL) {
  //::printf("File0\n", this);
}


File::~File() {
  //::printf("~File %d\n", this);
  //close();
}

//File::File(const char *_filename, const char * _mode, BridgeClass &b){
  
File::File(const char *_filename, const char * _mode){
  ::printf("File filename=%s  mode=%s\n", _filename, _mode);
  _file = NULL;
  _dir = NULL;
  _name = NULL;
  struct stat _st = {0};
  boolean exists = stat(_filename, &_st) == 0;
  //if(!exists || (!S_ISDIR(_st.st_mode) && !S_ISREG(_st.st_mode))){
  //  ::printf("Bad Entry[%s]: exists:%u, mode:0x%06X\n",_filename , exists, _st.st_mode);
  //  return;
  //}


  //_name = (char *)malloc(strlen(_filename)+1);
  ///strcpy(_name, _filename);
  //maybe not for folders?
  //if(S_ISREG(_st.st_mode)){
    //::printf("regular file...\n");
  _file = fopen(_filename, _mode);
  ::printf("_fopen=%d\n", _file);
  //} else {
  //  ::printf("folder...\n");
  //  _dir = opendir(_filename);
  //}
}

const char *File::name() {
  if( (fileno(_file) == 0) && (dirfd(_dir) == 0) ) return 0;
  return _name;
}

File::operator bool() {
  return ( (fileno(_file) != 0) || (dirfd(_dir) != 0) );
}


size_t File::write(const char *buf){
  if( fileno(_file) == 0) {
    return 0;
    ::printf("file write error: file not open!\n");   
  }
  return fwrite(buf, 1, strlen(buf), _file);
} 
    
size_t File::write(const uint8_t *buf, size_t size) {
  if( fileno(_file) == 0) {    
    ::printf("file write error: file not open!\n");
    return 0;
  }
  return fwrite(buf, 1, size, _file);
}

size_t File::write(uint8_t c) {
  if( fileno(_file) == 0) {    
    ::printf("file write error: file not open!\n");
    return 0;
  }    
  return write(&c, 1);
}

void File::flush() {
  if( fileno(_file) == 0) return;
  fflush(_file);
}

int File::read(void *buff, uint16_t nbyte) {
  if( fileno(_file) == 0) {    
    ::printf("file read error: file not open!\n");  
    return -1;
  }
  return ::fread(buff, 1, nbyte, _file);
}

int File::read() {
  if( fileno(_file) == 0) {    
    ::printf("file read error: file not open!\n");      
    return -1;
  }
  return getc(_file);
}

int File::peek() {
  if( fileno(_file) == 0) return -1;
  size_t pos = position();
  int c = getc(_file);
  if(c >= 0)
    fseek(_file, pos, ftell(_file));
  return c;
}

int File::available() {
  if( fileno(_file) == 0) return 0;
  return size() - position();
}

boolean File::seek(uint32_t position) {//SEEK_CUR, SEEK_END, and SEEK_SET
  if( fileno(_file) == 0) return false;
  return fseek(_file, position, 0) != -1; //seek to position from 0
}

uint32_t File::position() {
  if( fileno(_file) == 0) return 0;
  return ftell(_file);
}

uint32_t File::size() {
  if( fileno(_file) == 0) return 0;
  struct stat s = {0};
  if (stat(_name, &s) == 0) {
    return s.st_size;
  }
  return 0;
}

void File::close() {
  if(_file != NULL) {
    //::printf("closing _file=%d\n", _file);  
    ::fclose(_file);
    _file = NULL;
  }
  if(_dir != NULL) {
    ::closedir(_dir);
    _dir = NULL;
  }
  if(_name != NULL) {    
    ::free(_name);
    _name = NULL;
  }  
}

boolean File::isDirectory() {
  return ( dirfd(_dir) != 0 );
}

File File::openNextFile(const char * mode){
  struct dirent *file = readdir(_dir);
  if(file == NULL)
    return File();
  if(file->d_type != DT_REG && file->d_type != DT_DIR)
    return openNextFile(mode);
  String fname = String(file->d_name);
  if(fname.equals(".") || fname.equals(".."))
    return openNextFile(mode);
  String name = String(_name);
  if(!name.endsWith("/"))
    name += "/";
  name += fname;
  return File(name.c_str(), mode);
}

void File::rewindDirectory(void){
  if( dirfd(_dir) == 0) return;    
  closedir(_dir);
  _dir = opendir(_name);
}

//(rename(oldname, newname) == 0)

int fmkdir(const char *dir, int mode){ return mkdir(dir, mode); }
int frmdir(const char *dir){ return rmdir(dir); }
int fremove(const char *path){ return remove(path); }

boolean FileSystemClass::begin() {
  return true;
}


boolean FileSystemClass::begin(int selectPin) {
  return true;
}

File FileSystemClass::open(const char *filename, const char * mode) {
  //::printf("open... %s:%s\n", filename, mode);  
  return(File(filename, mode));
}

boolean FileSystemClass::exists(const char *filepath) {
  struct stat _stat = {0};
  boolean exists = stat(filepath, &_stat) == 0;
  return exists && (S_ISDIR(_stat.st_mode) || S_ISREG(_stat.st_mode));
}

boolean FileSystemClass::mkdir(const char *filepath) {
  struct stat _stat = {0};
  if (stat(filepath, &_stat) == -1) {
    return fmkdir(filepath, 0700) == 0;
  }
  return false;
}

boolean FileSystemClass::remove(const char *filepath) {
  struct stat _stat = {0};
  if (stat(filepath, &_stat) == 0) {
    return fremove(filepath) == 0;
  }
  return false;
}

boolean FileSystemClass::rmdir(const char *filepath) {
  struct stat _stat = {0};
  if (stat(filepath, &_stat) == 0) {
    return frmdir(filepath) == 0;
  }
  return false;
}

FileSystemClass FileSystem;

}
