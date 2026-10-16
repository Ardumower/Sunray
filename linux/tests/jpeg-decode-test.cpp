#include "../../sunray/src/camera/SafeJpeg.h"
#include <cassert>
#include <cstring>
#include <iostream>
static std::vector<uint8_t> makeJpeg(bool gray=false) {
  jpeg_compress_struct c{}; jpeg_error_mgr e{}; c.err=jpeg_std_error(&e);
  jpeg_create_compress(&c); unsigned char* bytes=nullptr; unsigned long size=0;
  jpeg_mem_dest(&c,&bytes,&size); c.image_width=16; c.image_height=16;
  c.input_components=gray?1:3; c.in_color_space=gray?JCS_GRAYSCALE:JCS_RGB;
  jpeg_set_defaults(&c); jpeg_start_compress(&c,TRUE);
  unsigned char row[48]; std::memset(row,128,sizeof(row));
  while(c.next_scanline<c.image_height){JSAMPROW r=row;jpeg_write_scanlines(&c,&r,1);}
  jpeg_finish_compress(&c);std::vector<uint8_t> result(bytes,bytes+size);
  jpeg_destroy_compress(&c);std::free(bytes);return result;
}
int main(){
  std::vector<uint8_t> rgb;int w=99,h=99;
  const auto valid=makeJpeg();const auto gray=makeJpeg(true);
  auto good=[&](const std::vector<uint8_t>& v){assert(SafeJpeg::decode(v.data(),v.size(),rgb,w,h));assert(w==16&&h==16&&rgb.size()==768);};
  auto bad=[&](const uint8_t* data,size_t size){assert(!SafeJpeg::decode(data,size,rgb,w,h));assert(w==0&&h==0&&rgb.empty());};
  bad(nullptr,10);bad(valid.data(),0);
  const std::vector<uint8_t> zeros(1024,0);
  const uint8_t brokenHeader[]={0xff,0xd8,0xff,0xc0,0,2,0xff,0xd9};
  for(int i=0;i<1000;i++){
    good(valid);bad(zeros.data(),zeros.size());good(gray);
    bad(brokenHeader,sizeof(brokenHeader));
  }
  for(size_t i=1;i<valid.size();i++){bad(valid.data(),i);good(valid);}
  auto oversized=valid;
  for(size_t i=0;i+8<oversized.size();i++)if(oversized[i]==0xff&&oversized[i+1]==0xc0){oversized[i+5]=0x20;oversized[i+6]=0;break;}
  bad(oversized.data(),oversized.size());good(valid);
  std::cout<<"PASS: invalid headers, zero frames, all truncations, oversized image, RGB/grayscale and repeated recovery\n";
}
