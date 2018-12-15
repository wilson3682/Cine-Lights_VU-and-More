#ifndef __INC_MUZIO_VU_16_H
#define __INC_MUZIO_VU_16_H
#include <Arduino.h>
#include <FastLED.h>
#include <math.h>
#include "vu15.h"

inline void vu_classic_centre() {
  vu15(true, 0);
}

#endif  // __INC_MUZIO_VU_16_H