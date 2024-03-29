#ifndef PRIVATE_KEY_H_
#define PRIVATE_KEY_H_
unsigned char example_key_DER[] = {
  0x30, 0x82, 0x02, 0x5c, 0x02, 0x01, 0x00, 0x02, 0x81, 0x81, 0x00, 0xc6,
  0x06, 0x6f, 0xc6, 0xdc, 0x86, 0x28, 0xa7, 0x0d, 0xdb, 0xca, 0xca, 0xf3,
  0x63, 0x27, 0x2f, 0x12, 0x39, 0x8f, 0x24, 0xf3, 0x74, 0xf2, 0xe3, 0x37,
  0x23, 0xe7, 0x6a, 0x69, 0xb0, 0x27, 0x1e, 0x21, 0x27, 0x67, 0xae, 0x42,
  0x88, 0x9c, 0x1c, 0x03, 0x84, 0x30, 0xbf, 0x5d, 0x1d, 0x21, 0x5c, 0x73,
  0x52, 0x31, 0x61, 0xb9, 0xe1, 0x92, 0x88, 0x64, 0x78, 0xad, 0xb1, 0x16,
  0x28, 0xa9, 0xb3, 0xbd, 0x6e, 0x43, 0x20, 0x0c, 0x08, 0xdc, 0x35, 0x22,
  0x0a, 0x2f, 0x9d, 0x7d, 0x63, 0xd8, 0x30, 0xd4, 0x0b, 0x0d, 0xaa, 0xf0,
  0x9f, 0x87, 0xc9, 0x16, 0x60, 0x06, 0xa0, 0x16, 0xd1, 0xfc, 0x3c, 0x23,
  0xd0, 0x4c, 0x5a, 0xa3, 0x0b, 0x04, 0x0c, 0x6c, 0x84, 0xd3, 0x1d, 0x59,
  0x9c, 0x3d, 0xb5, 0x32, 0x97, 0x50, 0x17, 0x3f, 0xcf, 0xf0, 0xe2, 0xec,
  0x72, 0xef, 0x6c, 0x25, 0xc2, 0xed, 0xe3, 0x02, 0x03, 0x01, 0x00, 0x01,
  0x02, 0x81, 0x80, 0x76, 0x08, 0x2a, 0x21, 0xd7, 0x19, 0xe4, 0x2b, 0x46,
  0x98, 0x66, 0x74, 0xb7, 0xc3, 0xb6, 0xfc, 0x58, 0x99, 0x94, 0x57, 0xcb,
  0x01, 0x3f, 0x30, 0xed, 0x91, 0xea, 0x02, 0xc4, 0x82, 0x29, 0x9e, 0xcc,
  0xd6, 0x26, 0xf7, 0x78, 0x52, 0xe2, 0xf1, 0xd3, 0xaa, 0xbc, 0x3f, 0xa3,
  0xe9, 0x94, 0x13, 0xfd, 0xec, 0xd4, 0xe0, 0x52, 0x22, 0x40, 0xec, 0x29,
  0x2e, 0xc1, 0x20, 0xd7, 0x7b, 0x5f, 0x42, 0x65, 0x45, 0x73, 0x15, 0x99,
  0x60, 0xeb, 0x53, 0xd6, 0xcd, 0xa8, 0x40, 0x1b, 0x95, 0xb6, 0xec, 0xc5,
  0x04, 0x95, 0x8a, 0xc1, 0x76, 0x39, 0xf6, 0x10, 0x9e, 0xe5, 0x17, 0xb2,
  0xab, 0x7a, 0x62, 0x97, 0x02, 0xe4, 0x32, 0x59, 0x4f, 0x45, 0x3d, 0x4a,
  0xe7, 0xf0, 0x8c, 0xfa, 0xc0, 0xe8, 0x99, 0x8a, 0x80, 0x40, 0x99, 0x26,
  0x6c, 0xde, 0x08, 0x38, 0x40, 0xf3, 0xa2, 0x12, 0xbb, 0x2a, 0x41, 0x02,
  0x41, 0x00, 0xf0, 0x18, 0x1b, 0xa7, 0xf7, 0xc7, 0x8a, 0xdc, 0x1d, 0xb8,
  0x54, 0x35, 0xbb, 0x4b, 0xe0, 0x7e, 0x64, 0xda, 0xf5, 0xd8, 0xc9, 0xb6,
  0x63, 0xc1, 0x86, 0xb6, 0x43, 0x6e, 0xbd, 0xc9, 0x1d, 0xb3, 0xce, 0xc1,
  0xfe, 0xf9, 0xa6, 0x9f, 0x98, 0xd9, 0x1d, 0x4c, 0x60, 0x73, 0xef, 0xab,
  0xbf, 0x77, 0x59, 0x51, 0x9f, 0xde, 0x3d, 0xc0, 0x1c, 0x8e, 0x98, 0x97,
  0x8a, 0xc3, 0xef, 0xb1, 0xbb, 0xe1, 0x02, 0x41, 0x00, 0xd3, 0x24, 0xdb,
  0x36, 0x47, 0x5e, 0x82, 0x51, 0x32, 0x9a, 0x9e, 0xd1, 0xd0, 0x97, 0x56,
  0x9f, 0x1b, 0xbf, 0x05, 0x85, 0x71, 0x6c, 0x9f, 0x04, 0x78, 0x7d, 0x7b,
  0x74, 0x3f, 0x65, 0x69, 0x3e, 0x6b, 0x49, 0xee, 0x6c, 0x62, 0xc4, 0xb3,
  0x44, 0x08, 0x77, 0x9b, 0x91, 0xc0, 0xee, 0x78, 0xea, 0xcd, 0x49, 0x85,
  0x82, 0xc2, 0xdc, 0x0f, 0x68, 0xa5, 0xec, 0xf1, 0xba, 0x57, 0xa7, 0x02,
  0x43, 0x02, 0x40, 0x63, 0x1a, 0xd0, 0x6f, 0xa9, 0x0b, 0xa8, 0xf9, 0xeb,
  0x1a, 0xa7, 0x47, 0xf6, 0xa3, 0xff, 0x6a, 0xac, 0xde, 0xe5, 0x14, 0x33,
  0x4f, 0x22, 0x26, 0x44, 0x20, 0xff, 0xfc, 0xba, 0x42, 0x46, 0x0c, 0x6e,
  0x90, 0x0d, 0x5b, 0xa7, 0xb7, 0xc1, 0x33, 0xfd, 0xb0, 0x05, 0x30, 0x56,
  0x02, 0x22, 0xea, 0x74, 0xe8, 0x08, 0x81, 0x88, 0x23, 0xc0, 0xa5, 0xeb,
  0xbe, 0xc1, 0xfc, 0xd6, 0xf8, 0x1a, 0x81, 0x02, 0x41, 0x00, 0xc9, 0x5c,
  0x08, 0xc2, 0x86, 0xe5, 0x96, 0x9a, 0x21, 0x0c, 0x5c, 0x11, 0xf4, 0x3f,
  0x9f, 0x98, 0x35, 0x39, 0xc2, 0xe6, 0x33, 0xf9, 0x46, 0xdd, 0x58, 0x9b,
  0x32, 0xb4, 0xf6, 0x10, 0x9c, 0x81, 0xae, 0x87, 0xdf, 0x1c, 0xf6, 0x44,
  0x68, 0x41, 0xa5, 0x61, 0x8b, 0xb7, 0x40, 0xab, 0x2c, 0x1d, 0xa0, 0x91,
  0x51, 0x06, 0x17, 0x17, 0x0e, 0x8a, 0xda, 0x52, 0x51, 0x65, 0x48, 0x5a,
  0x39, 0x49, 0x02, 0x40, 0x61, 0x07, 0xdf, 0x23, 0x44, 0x50, 0xaa, 0xf6,
  0x51, 0x7f, 0x1c, 0xc6, 0xcc, 0x4f, 0x9d, 0x53, 0xf7, 0x9e, 0x0a, 0x18,
  0x52, 0xb1, 0x7b, 0x20, 0x88, 0x36, 0x73, 0x9b, 0xb2, 0x43, 0xa2, 0x1b,
  0x95, 0x45, 0x80, 0x58, 0x5b, 0x7b, 0x9d, 0xd9, 0x97, 0x7c, 0x51, 0xcd,
  0x0c, 0x3a, 0x32, 0x72, 0x74, 0x5a, 0x78, 0x07, 0xbd, 0xbf, 0x18, 0x19,
  0x77, 0xb3, 0xf5, 0x2b, 0x59, 0xd6, 0x43, 0x9e
};
unsigned int example_key_DER_len = 608;
#endif
