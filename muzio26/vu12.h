#ifndef __INC_MUZIO_VU_12_H
#define __INC_MUZIO_VU_12_H
#include <Arduino.h>
#include <math.h>
#include "common.h"
#include "util.h"

////////////////////////////////////
/// Half peak blue dots
////////////////////////////////////
void vu_half_peak_blue_dots()
{
    unsigned long startMillis = millis(); // Start of sample window
    float peakToPeak = 0;                 // peak-to-peak level
    unsigned int signalMax = 0;
    unsigned int signalMin = 1023;
    unsigned int c, y;

    while (millis() - startMillis < SAMPLE_WINDOW)
    {
        sample = analogRead(MIC_PIN);
        if (sample < 1024)
        {
            if (sample > signalMax)
            {
                signalMax = sample;
            }
            else if (sample < signalMin)
            {
                signalMin = sample;
            }
        }
    }
    peakToPeak = signalMax - signalMin;

    for (int i = 0; i <= N_PIXELS_HALF - 1; i++)
    {
        CRGB color = Wheel2(map(i, 0, N_PIXELS_HALF - 1, 30, 150));
        ledsLeft[LAST_PIXEL_OFFSET - i] = color;
        ledsLeft[0 + i] = color;
    }

    c = fscale(INPUT_FLOOR, INPUT_CEILING, N_PIXELS_HALF, 0, peakToPeak, 2);

    if (c < peakLeft)
    {
        peakLeft = c;         // Keep dot on top
        dotHangCount = 0; // make the dot hang before falling
    }
    if (c <= N_PIXELS) // Fill partial column with off pixels
    {
        drawLine(N_PIXELS_HALF, N_PIXELS_HALF - c, CRGB::Black);
        drawLine(N_PIXELS_HALF, N_PIXELS_HALF + c, CRGB::Black);
    }

    y = min(0, N_PIXELS_HALF - peakLeft);
    CRGB color1 = Wheel2(map(y, 0, N_PIXELS_HALF - 1, 30, 150));
    ledsLeft[y - 1] = color1;

    y = max(N_PIXELS_HALF + peakLeft, LAST_PIXEL_OFFSET);
    ledsLeft[y] = color1;

    FastLED.show();

    // Frame based peak dot animation
    if (dotHangCount > PEAK_HANG)
    { //Peak pause length
        if (++dotCount >= PEAK_FALL2)
        { //Fall rate
            peak++;
            dotCount = 0;
        }
    }
    else
    {
        dotHangCount++;
    }
}

#endif // __INC_MUZIO_VU_12_H