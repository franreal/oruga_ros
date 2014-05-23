#ifndef SERIALCOMM_UTILS_H
#define SERIALCOMM_UTILS_H

namespace utils {

  inline unsigned int int16ToBuf(int16_t x, unsigned char* buf, unsigned int* checksum) {
    *buf = (unsigned char)x;
    *checksum += *buf;
    buf++;

    *buf = (unsigned char)(x >> 8);
    *checksum += *buf;

    return 2;
  }

};

#endif  // SERIALCOMM_UTILS_H

