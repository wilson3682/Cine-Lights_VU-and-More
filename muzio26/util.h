#ifndef __INC_MUZIO_UTIL_H
#define __INC_MUZIO_UTIL_H
#include <FastLED.h>
#include <math.h>
#include "common.h"

inline uint32_t colorToInt(uint8_t r, uint8_t g, uint8_t b)
{
  return ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
}

inline uint32_t colorToInt(CRGB c)
{
  return ((uint32_t)c.r << 16) | ((uint32_t)c.g << 8) | c.b;
}

CRGB Wheel2(byte WheelPos)
{
  if (WheelPos < 85)
  {
    return CRGB(WheelPos * 3, 255 - WheelPos * 3, 0);
  }
  else if (WheelPos < 170)
  {
    WheelPos -= 85;
    return CRGB(255 - WheelPos * 3, 0, WheelPos * 3);
  }
  else
  {
    WheelPos -= 170;
    return CRGB(0, WheelPos * 3, 255 - WheelPos * 3);
  }
}

// Input a value 0 to 255 to get a color value.
// The colors are a transition r - g - b - back to r.
// TODO: Remove when neop is removed
inline uint32_t Wheel(byte WheelPos)
{
  return colorToInt(Wheel2(WheelPos));
}

float fscale(float originalMin, float originalMax, float newBegin, float newEnd, float inputValue, float curve)
{
  float OriginalRange = 0;
  float NewRange = 0;
  float zeroRefCurVal = 0;
  float normalizedCurVal = 0;
  float rangedValue = 0;
  boolean invFlag = 0;

  // condition curve parameter
  // limit range

  if (curve > 10)
    curve = 10;
  if (curve < -10)
    curve = -10;

  curve = (curve * -.1);  // - invert and scale - this seems more intuitive - postive numbers give more weight to high end on output
  curve = pow(10, curve); // convert linear scale into lograthimic exponent for other pow function

  // Check for out of range inputValues
  if (inputValue < originalMin)
  {
    inputValue = originalMin;
  }
  if (inputValue > originalMax)
  {
    inputValue = originalMax;
  }

  // Zero Refference the values
  OriginalRange = originalMax - originalMin;

  if (newEnd > newBegin)
  {
    NewRange = newEnd - newBegin;
  }
  else
  {
    NewRange = newBegin - newEnd;
    invFlag = 1;
  }

  zeroRefCurVal = inputValue - originalMin;
  normalizedCurVal = zeroRefCurVal / OriginalRange; // normalize to 0 - 1 float

  // Check for originalMin > originalMax  - the math for all other cases i.e. negative numbers seems to work out fine
  if (originalMin > originalMax)
  {
    return 0;
  }

  if (invFlag == 0)
  {
    rangedValue = (pow(normalizedCurVal, curve) * NewRange) + newBegin;
  }
  else // invert the ranges
  {
    rangedValue = newBegin - (pow(normalizedCurVal, curve) * NewRange);
  }

  return rangedValue;
}

////////////////////////////////////////
/////// addGlitter // Let's add some glitter, thanks to Mark
////////////////////////////////////////
void addGlitter(fract8 chanceOfGlitter)
{

  if (random8() < chanceOfGlitter)
  {
    ledsLeft[random16(N_PIXELS)] += CRGB::White;
  }
}

void drawLine(uint8_t from, uint8_t to, CRGB c)
{
  if (from > to)
  {
    uint8_t fromTemp;
    fromTemp = from;
    from = to;
    to = fromTemp;
  }
  from = max(0, from);
  to = min(to, LAST_PIXEL_OFFSET);
  for (int i = from; i <= to; i++)
  {
    ledsLeft[i] = c;
  }
}

//Used to draw a line between two points of a given color
void drawLine(uint8_t from, uint8_t to, uint32_t c)
{
  drawLine(from, to, CRGB(c));
}

/*
   Function for averaging the sample readings
*/
void averageReadings(uint8_t channel)
{

  uint16_t minLvl, maxLvl;

  // minLvl and maxLvl indicate the volume range over prior frames, used
  // for vertically scaling the output graph (so it looks interesting
  // regardless of volume level).  If they're too close together though
  // (e.g. at very low volume levels) the graph becomes super coarse
  // and 'jumpy'...so keep some minimum distance between them (this
  // also lets the graph go to zero when no sound is playing):
  if (channel == 0)
  {
    minLvl = maxLvl = volLeft[0];
    for (int i = 1; i < SAMPLES; i++)
    {
      if (volLeft[i] < minLvl)
        minLvl = volLeft[i];
      else if (volLeft[i] > maxLvl)
        maxLvl = volLeft[i];
    }
    if ((maxLvl - minLvl) < TOP)
      maxLvl = minLvl + TOP;

    minLvlAvgLeft = (minLvlAvgLeft * 63 + minLvl) >> 6; // Dampen min/max levels
    maxLvlAvgLeft = (maxLvlAvgLeft * 63 + maxLvl) >> 6; // (fake rolling average)
  }
}

int readMic(int *height)
{
  int n;
  
  n = analogRead(MIC_PIN); // Raw reading from mic
  n = abs(n - 512 - DC_OFFSET);       // Center on zero
  n = (n <= NOISE) ? 0 : (n - NOISE); // Remove noise/hum
  lvlLeft = ((lvlLeft * 7) + n) >> 3; // "Dampened" reading (else looks twitchy)

  // Calculate bar height based on dynamic min/max levels (fixed point):
  *height = TOP * (lvlLeft - minLvlAvgLeft) / (long)(maxLvlAvgLeft - minLvlAvgLeft);

  if (*height < 0L)
    *height = 0; // Clip output
  else if (*height > TOP)
    *height = TOP;
  if (*height > peakLeft)
    peakLeft = *height; // Keep 'peak' dot at top

  return n;
}

#endif // __INC_MUZIO_UTIL_H