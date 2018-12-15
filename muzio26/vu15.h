#ifndef __INC_MUZIO_VU_15_H
#define __INC_MUZIO_VU_15_H
#include <Arduino.h>
#include <FastLED.h>
#include "common.h"

/*
   Old-skool green and red from bottom or middle
*/
// ------------------
// -- VU functions --
// ------------------

uint16_t auxReading(uint8_t channel)
{
  uint16_t height = 0;

  if (channel == 0)
  {
    int n = analogRead(MIC_PIN);        // Raw reading from left line in
    n = abs(n - 512 - DC_OFFSET);       // Center on zero
    n = (n <= NOISE) ? 0 : (n - NOISE); // Remove noise/hum
    lvlLeft = ((lvlLeft * 7) + n) >> 3; // "Dampened" reading (else looks twitchy)
    volLeft[volCountLeft] = n;          // Save sample for dynamic leveling
    volCountLeft++;
    volCountLeft = volCountLeft % SAMPLES;
    // Calculate bar height based on dynamic min/max levels (fixed point):
    height = TOP * (lvlLeft - minLvlAvgLeft) / (long)(maxLvlAvgLeft - minLvlAvgLeft);
  }

  // Calculate bar height based on dynamic min/max levels (fixed point):
  height = constrain(height, 0, TOP);

  return height;
}

/*
   Function for dropping the peak
*/
//uint8_t peakLeft, peakRight;
void dropPeak(uint8_t channel)
{

  static uint8_t dotCountLeft;

  if (channel == 0)
  {
    if (++dotCountLeft >= PEAK_FALL)
    { //fall rate
      if (peakLeft > 0)
        peakLeft--;
      dotCountLeft = 0;
    }
  }
}

void vu15(bool is_centered, uint8_t channel)
{
  CRGB *leds;

  // uint8_t i = 0;
  uint8_t *peak; // Pointer variable declaration
  uint16_t height = auxReading(channel);

  if (channel == 0)
  {
    leds = ledsLeft;  // Store address of peakLeft in peak, then use *peak to
    peak = &peakLeft; // access the value of that address
  }

  if (height > *peak)
    *peak = height; // Keep 'peak' dot at top

  if (is_centered)
  {
    // Color pixels based on old school green / red
    for (uint8_t i = 0; i < N_PIXELS_HALF; i++)
    {
      if (i >= height)
      {
        // Pixels greater than peak, no light
        leds[N_PIXELS_HALF - i - 1] = CRGB::Black;
        leds[N_PIXELS_HALF + i] = CRGB::Black;
      }
      else
      {
        if (i > N_PIXELS_HALF - (N_PIXELS_HALF / 3))
        {
          leds[N_PIXELS_HALF - i - 1] = CRGB::Red;
          leds[N_PIXELS_HALF + i] = CRGB::Red;
        }
        else
        {
          leds[N_PIXELS_HALF - i - 1] = CRGB::Green;
          leds[N_PIXELS_HALF + i] = CRGB::Green;
        }
      }
    }

    // Draw peak dot
    if (*peak > 0 && *peak <= N_PIXELS_HALF - 1)
    {
      if (*peak > N_PIXELS_HALF - (N_PIXELS_HALF / 3))
      {
        leds[N_PIXELS_HALF - *peak - 1] = CRGB::Red;
        leds[N_PIXELS_HALF + *peak] = CRGB::Red;
      }
      else
      {
        leds[N_PIXELS_HALF - *peak - 1] = CRGB::Green;
        leds[N_PIXELS_HALF + *peak] = CRGB::Green;
      }
    }
  }
  else
  {
    // Color pixels based on old school green/red vu
    for (uint8_t i = 0; i < N_PIXELS; i++)
    {
      if (i >= height)
        leds[i] = CRGB::Black;
      else if (i > N_PIXELS - (N_PIXELS / 3))
        leds[i] = CRGB::Red;
      else
        leds[i] = CRGB::Green;
    }

    // Draw peak dot
    if (*peak > 0 && *peak <= N_PIXELS - 1)
    {
      if (*peak > N_PIXELS - (N_PIXELS / 3))
        leds[*peak] = CRGB::Red;
      else
        leds[*peak] = CRGB::Green;
    }
  } // switch step

  dropPeak(channel);

  averageReadings(channel);

  FastLED.show();
}

inline void vu_classic_normal() {
  vu15(false, 0); // classic normal
}

#endif // __INC_MUZIO_VU_15_H