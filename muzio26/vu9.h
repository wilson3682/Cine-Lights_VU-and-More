#ifndef __INC_MUZIO_VU_9_H
#define __INC_MUZIO_VU_9_H
#include <Arduino.h>
#include "common.h"
#include "util.h"

CRGBPalette16 currentPalette(OceanColors_p);
CRGBPalette16 targetPalette(CloudColors_p);

void soundble()
{ // Quick and dirty sampling of the microphone.

  int tmp = analogRead(MIC_PIN) - 512 - DC_OFFSET;
  sample = abs(tmp);

} // soundmems()

void sndwave()
{
  ledsLeft[N_PIXELS / 2] = ColorFromPalette(currentPalette, sample, sample * 2, currentBlending); // Put the sample into the center

  for (int i = N_PIXELS - 1; i > N_PIXELS / 2; i--)
  { //move to the left      // Copy to the left, and let the fade do the rest.
    ledsLeft[i] = ledsLeft[i - 1];
  }

  for (int i = 0; i < N_PIXELS / 2; i++)
  { // move to the right    // Copy to the right, and let the fade to the rest.
    ledsLeft[i] = ledsLeft[i + 1];
  }
  addGlitter(sampleavg);
}

void vu_pulse()
{
  //currentBlending = LINEARBLEND;
  currentPalette = OceanColors_p; // Initial palette.
  currentBlending = LINEARBLEND;
  EVERY_N_SECONDS(5)
  { 
    for (int i = 0; i < 16; i++)
    {
      targetPalette[i] = CHSV(random8(), 255, 255);
    }
  }

  EVERY_N_MILLISECONDS(100)
  { // AWESOME palette blending capability once they do change.
    uint8_t maxChanges = 24;
    nblendPaletteTowardPalette(currentPalette, targetPalette, maxChanges);
  }

  EVERY_N_MILLIS_I(thistimer, 20)
  {                                         // For fun, let's make the animation have a variable rate.
    uint8_t timeval = beatsin8(10, 20, 50); // Use a sinewave for the line below. Could also use peak/beat detection.
    thistimer.setPeriod(timeval);           // Allows you to change how often this routine runs.
    fadeToBlackBy(ledsLeft, N_PIXELS, 16);  // 1 = slow, 255 = fast fade. Depending on the faderate, the LED's further away will fade out.
    sndwave();
    soundble();
    //  }
    //   // Copy left LED array into right LED array
    //  for (uint8_t i = 0; i < N_PIXELS; i++) {
    //    ledsRight[i] = ledsLeft[i];
  }
  FastLED.setBrightness(max_bright);
  FastLED.show();

} // loop()

#endif // __INC_MUZIO_VU_9_H