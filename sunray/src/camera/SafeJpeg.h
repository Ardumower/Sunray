#pragma once
#ifdef __linux__
#include <cstddef>
#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <csetjmp>
#include <vector>
#include <jpeglib.h>

namespace SafeJpeg {
// Keep libjpeg's mutable state on the heap: automatic variables modified
// after setjmp would have indeterminate values following longjmp.
struct Decoder {
  jpeg_decompress_struct info;
  jpeg_error_mgr error;
  std::jmp_buf recovery;
};
inline void fail(j_common_ptr info) {
  Decoder* decoder = static_cast<Decoder*>(info->client_data);
  std::longjmp(decoder->recovery, 1);
}
inline void message(j_common_ptr info, int level) {
  // libjpeg may repair truncated/corrupt input with a warning. Such a frame
  // must not be used for robot guidance either.
  if (level < 0) fail(info);
}
inline bool decode(const uint8_t* data, size_t size, std::vector<uint8_t>& rgb,
                   int& width, int& height) {
  rgb.clear(); width = 0; height = 0;
  if (!data || size == 0) return false;
  Decoder* const decoder = static_cast<Decoder*>(std::calloc(1, sizeof(Decoder)));
  if (!decoder) return false;
  decoder->info.err = jpeg_std_error(&decoder->error);
  decoder->error.error_exit = fail;
  decoder->error.emit_message = message;
  decoder->info.client_data = decoder;
  if (setjmp(decoder->recovery)) {
    jpeg_destroy_decompress(&decoder->info);
    std::free(decoder);
    rgb.clear(); width = 0; height = 0;
    return false;
  }
  jpeg_create_decompress(&decoder->info);
  jpeg_mem_src(&decoder->info, data, size);
  if (jpeg_read_header(&decoder->info, TRUE) != JPEG_HEADER_OK ||
      !decoder->info.image_width || !decoder->info.image_height ||
      decoder->info.image_width > 4096 || decoder->info.image_height > 4096) {
    jpeg_destroy_decompress(&decoder->info); std::free(decoder); return false;
  }
  decoder->info.out_color_space = JCS_RGB;
  jpeg_start_decompress(&decoder->info);
  if (decoder->info.output_components != 3) {
    jpeg_destroy_decompress(&decoder->info); std::free(decoder); return false;
  }
  try {
    rgb.resize(static_cast<size_t>(decoder->info.output_width) * decoder->info.output_height * 3);
  } catch (...) {
    jpeg_destroy_decompress(&decoder->info); std::free(decoder); return false;
  }
  while (decoder->info.output_scanline < decoder->info.output_height) {
    JSAMPROW row = rgb.data() + static_cast<size_t>(decoder->info.output_scanline) * decoder->info.output_width * 3;
    if (jpeg_read_scanlines(&decoder->info, &row, 1) != 1) {
      jpeg_destroy_decompress(&decoder->info); std::free(decoder); rgb.clear(); return false;
    }
  }
  jpeg_finish_decompress(&decoder->info);
  width = decoder->info.output_width; height = decoder->info.output_height;
  jpeg_destroy_decompress(&decoder->info);
  std::free(decoder);
  return true;
}
} // namespace SafeJpeg
#endif
