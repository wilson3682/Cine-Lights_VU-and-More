#ifndef __INC_MUZIO_VU_5_H
#define __INC_MUZIO_VU_5_H
#include <Arduino.h>
#include "common.h"
#include "util.h"

void vu_shooting_star()
{
  uint8_t i;
  uint16_t minLvlLeft, maxLvlLeft;
  int n, height;

  n = readMic(&height);

#ifdef CENTERED
  // Color pixels based on rainbow gradient
  for (i = 0; i < (N_PIXELS / 2); i++)
  {
    if (((N_PIXELS / 2) + i) >= height)
    {
      ledsLeft[N_PIXELS_HALF + i] = CRGB::Black;
      ledsLeft[N_PIXELS_HALF - i] = CRGB::Black;
    }
    else
    {
      ledsLeft[N_PIXELS_HALF + i] = Wheel2(map(((N_PIXELS / 2) + i), 0, LAST_PIXEL_OFFSET, 30, 150));
      ledsLeft[N_PIXELS_HALF - i] = Wheel2(map(((N_PIXELS / 2) - i), 0, LAST_PIXEL_OFFSET, 30, 150));
    }
  }

  // Draw peak dot
  if (peakLeft > 0 && peakRight <= LAST_PIXEL_OFFSET)
  {
    ledsLeft[N_PIXELS_HALF + peakLeft], = CRGB:White;
    ledsLeft[N_PIXELS_HALF - peakLeft] = CRGB:White;
  }
#else
  // Color pixels based on rainbow gradient
  for (i = 0; i < N_PIXELS; i++)
  {
    ledsLeft[i] = i >= height ? CRGB::Black : Wheel2(map(i, 0, N_PIXELS - 1, 30, 150));
  }

  // Draw peak dot
  if (peakLeft > 0 && peakLeft <= LAST_PIXEL_OFFSET)
  {
    ledsLeft[peakLeft] = CRGB::White;
  }

#endif

  // Every few frames, make the peak pixel drop by 1:
  EVERY_N_MILLISECONDS(PEAK_FALL_MILLIS)
  {
    FastLED.show(); // Update strip

    //fall rate
    if (peakLeft > 0)
      peakLeft--;
  }

  volLeft[volCountLeft] = n; // Save sample for dynamic leveling
  if (++volCountLeft >= SAMPLES)
    volCountLeft = 0; // Advance/rollover sample counter

  // Get volume range of prior frames
  minLvlLeft = maxLvlLeft = vol[0];
  for (i = 1; i < SAMPLES; i++)
  {
    if (volLeft[i] < minLvlLeft)
      minLvlLeft = volLeft[i];
    else if (volLeft[i] > maxLvlLeft)
      maxLvlLeft = volLeft[i];
  }
  // minLvl and maxLvl indicate the volume range over prior frames, used
  // for vertically scaling the output graph (so it looks interesting
  // regardless of volume level).  If they're too close together though
  // (e.g. at very low volume levels) the graph becomes super coarse
  // and 'jumpy'...so keep some minimum distance between them (this
  // also lets the graph go to zero when no sound is playing):
  if ((maxLvlLeft - minLvlLeft) < TOP)
    maxLvlLeft = minLvlLeft + TOP;
  minLvlAvgLeft = (minLvlAvgLeft * 63 + minLvlLeft) >> 6; // Dampen min/max levels
  maxLvlAvgLeft = (maxLvlAvgLeft * 63 + maxLvlLeft) >> 6; // (fake rolling average)
}

#endif // __INC_MUZIO_VU_5_H