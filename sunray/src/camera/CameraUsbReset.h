#pragma once
#include <string>
#include <fstream>
#include <cstdlib>
#include <cstdio>
#include <cerrno>
#include <climits>
#include <sys/stat.h>
#include <sys/sysmacros.h>
#include <sys/ioctl.h>
#include <linux/usbdevice_fs.h>
#include <fcntl.h>
#include <unistd.h>

namespace CameraUsbReset {
// Resolve only the USB device owning this V4L2 descriptor, never a USB hub.
inline std::string deviceFor(int fd){
  struct stat st{};if(fstat(fd,&st)<0 || !S_ISCHR(st.st_mode))return {};
  char link[128],resolved[PATH_MAX];
  snprintf(link,sizeof(link),"/sys/dev/char/%u:%u/device",major(st.st_rdev),minor(st.st_rdev));
  if(!realpath(link,resolved))return {};
  std::string path=resolved;
  while(path.size()>1){
    unsigned bus=0,dev=0;std::string cls;
    if((std::ifstream(path+"/busnum")>>bus) && (std::ifstream(path+"/devnum")>>dev)){
      if(!(std::ifstream(path+"/bDeviceClass")>>cls) || cls=="09" || !bus || !dev)return {};
      char node[64];snprintf(node,sizeof(node),"/dev/bus/usb/%03u/%03u",bus,dev);return node;
    }
    const auto slash=path.find_last_of('/');if(slash==std::string::npos)break;path.resize(slash);
  }
  return {};
}
inline int reset(const std::string& device){
  if(device.empty())return ENODEV;
  int fd=open(device.c_str(),O_WRONLY|O_CLOEXEC);if(fd<0)return errno;
  int error=ioctl(fd,USBDEVFS_RESET,0)<0?errno:0;
  close(fd);return error;
}
}
