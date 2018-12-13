#ifndef __INC_MUZIO_VU_7_H
#define __INC_MUZIO_VU_7_H
#include <Arduino.h>
#include "common.h"
#include "util.h"

void soundmems()
{ // Rolling average counter - means we don't have to go through an array each time.
  newtime = millis();
  int tmp = analogRead(MIC_PIN) - 512;
  sample = abs(tmp);

  int potin = map(analogRead(POT_PIN), 0, 1023, 0, 60);

  samplesum = samplesum + sample - samplearray[samplecount]; // Add the new sample and remove the oldest sample in the array
  sampleavg = samplesum / NSAMPLES;                          // Get an average
  samplearray[samplecount] = sample;                         // Update oldest sample in the array with new sample
  samplecount = (samplecount + 1) % NSAMPLES;                // Update the counter for the array

  if (newtime > (oldtime + 200))
    digitalWrite(13, LOW); // Turn the LED off 200ms after the last peak.

  if ((sample > (sampleavg + potin)) && (newtime > (oldtime + 60)))
  { // Check for a peak, which is 30 > the average, but wait at least 60ms for another.
    step = -1;
    peakcount++;
    digitalWrite(13, HIGH);
    oldtime = newtime;
  }
} // soundmems()

void ripple3()
{
  for (int i = 0; i < N_PIXELS; i++)
    ledsLeft[i] = CHSV(bgcol, 255, sampleavg * 2); // Set the background colour.

  switch (step)
  {

  case -1: // Initialize ripple variables.
    center = random(N_PIXELS);
    colour = (peakspersec * 10) % 255; // More peaks/s = higher the hue colour.
    step = 0;
    bgcol = bgcol + 8;
    break;

  case 0:
    ledsLeft[center] = CHSV(colour, 255, 255); // Display the first pixel of the ripple.
    step++;
    break;

  case maxsteps: // At the end of the ripples.
    // step = -1;
    break;

  default:                                                                                   // Middle of the ripples.
    ledsLeft[(center + step + N_PIXELS) % N_PIXELS] += CHSV(colour, 255, myfade / step * 2); // Simple wrap from Marc Miller.
    ledsLeft[(center - step + N_PIXELS) % N_PIXELS] += CHSV(colour, 255, myfade / step * 2);
    step++; // Next step.
    break;

    //  }
    //   // Copy left LED array into right LED array
    //  for (uint8_t i = 0; i < N_PIXELS; i++) {
    //    ledsRight[i] = ledsLeft[i];
  } // switch step
} 

// ripple()
void vu_ripple()
{

  EVERY_N_MILLISECONDS(1000)
  {
    peakspersec = peakcount; // Count the peaks per second. This value will become the foreground hue.
    peakcount = 0;           // Reset the counter every second.
  }

  soundmems();
  //soundmems_2();
  EVERY_N_MILLISECONDS(20)
  {
    ripple3();
  }

  FastLED.show();

} // loop()


#endif // __INC_MUZIO_VU_7_H