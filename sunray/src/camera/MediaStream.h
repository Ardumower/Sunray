// Simple media stream header for binary WS frames
// Layout (big endian):
//  0..3   magic 'SRBX'
//  4      version (1)
//  5      mediaType (1=camera_jpeg)
//  6..7   flags (uint16)
//  8..9   width (uint16)
// 10..11  height (uint16)
// 12..15  timestampMs (uint32)

#pragma once
#include <cstdint>

struct MediaHeader {
  uint32_t magic;      // 'SRBX'
  uint8_t version;     // 1
  uint8_t type;        // 1 = camera_jpeg
  uint16_t flags;      // reserved
  uint16_t width;      // pixels
  uint16_t height;     // pixels
  uint32_t timestamp;  // ms
};

inline void write_be16(uint8_t* p, uint16_t v) {
  p[0] = (uint8_t)((v >> 8) & 0xFF);
  p[1] = (uint8_t)(v & 0xFF);
}
inline void write_be32(uint8_t* p, uint32_t v) {
  p[0] = (uint8_t)((v >> 24) & 0xFF);
  p[1] = (uint8_t)((v >> 16) & 0xFF);
  p[2] = (uint8_t)((v >> 8) & 0xFF);
  p[3] = (uint8_t)(v & 0xFF);
}

inline void buildMediaHeader(uint8_t* out16, uint8_t mediaType, uint16_t width, uint16_t height, uint32_t tsMs, uint16_t flags = 0) {
  // magic 'SRBX'
  out16[0] = 'S'; out16[1] = 'R'; out16[2] = 'B'; out16[3] = 'X';
  out16[4] = 1; // version
  out16[5] = mediaType;
  write_be16(out16 + 6, flags);
  write_be16(out16 + 8, width);
  write_be16(out16 + 10, height);
  write_be32(out16 + 12, tsMs);
}

