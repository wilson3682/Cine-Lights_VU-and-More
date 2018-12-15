#ifndef __INC_MUZIO_VU_8_H
#define __INC_MUZIO_VU_8_H
#include <Arduino.h>
#include "common.h"
#include "util.h"

//vu 8 variables
int
    origin = 0,
    color_wait_count = 0,
    scroll_color = COLOR_MIN,
    last_intensity = 0,
    intensity_max = 0,
    origin_at_flip = 0;
uint32_t
    draw[DRAW_MAX];
boolean
    growing = false,
    fall_from_left = true;

int calculateIntensity()
{
    int intensity;
    // int      n, height;
    reading = analogRead(MIC_PIN);                        // Raw reading from mic
    reading = abs(reading - 512 - DC_OFFSET);             // Center on zero
    reading = (reading <= NOISE) ? 0 : (reading - NOISE); // Remove noise/hum
    lvlLeft = ((lvlLeft * 7) + reading) >> 3;             // "Dampened" reading (else looks twitchy)

    // Calculate bar height based on dynamic min/max levels (fixed point):
    // intensity = DRAW_MAX * (lvl - minLvlAvg) / (long)(maxLvlAvg - minLvlAvg);

    //  reading   = analogRead(MIC_PIN_2);                        // Raw reading from mic
    //  reading   = abs(reading - 512 - DC_OFFSET); // Center on zero
    //  reading   = (reading <= NOISE) ? 0 : (reading - NOISE);             // Remove noise/hum
    //  lvlRight = ((lvlRight * 7) + reading) >> 3;    // "Dampened" reading (else looks twitchy)
    //
    // Calculate bar height based on dynamic min/max levels (fixed point):
    intensity = DRAW_MAX * (lvlLeft - minLvlAvgLeft) / (long)(maxLvlAvgLeft - minLvlAvgLeft);
    //
    //  // Calculate bar height based on dynamic min/max levels (fixed point):
    //  intensity = DRAW_MAX * (lvlRight - minLvlAvgRight) / (long)(maxLvlAvgRight - minLvlAvgRight);

    return constrain(intensity, 0, DRAW_MAX - 1);
}

void updateOrigin(int intensity)
{
    // detect peak change and save origin at curve vertex
    if (growing && intensity < last_intensity)
    {
        growing = false;
        intensity_max = last_intensity;
        fall_from_left = !fall_from_left;
        origin_at_flip = origin;
    }
    else if (intensity > last_intensity)
    {
        growing = true;
        origin_at_flip = origin;
    }
    last_intensity = intensity;

    // adjust origin if falling
    if (!growing)
    {
        if (fall_from_left)
        {
            origin = origin_at_flip + ((intensity_max - intensity) / 2);
        }
        else
        {
            origin = origin_at_flip - ((intensity_max - intensity) / 2);
        }
        // correct for origin out of bounds
        if (origin < 0)
        {
            origin = DRAW_MAX - abs(origin);
        }
        else if (origin > DRAW_MAX - 1)
        {
            origin = origin - DRAW_MAX - 1;
        }
    }
}

void assignDrawValues(int intensity)
{
    // draw amplitue as 1/2 intensity both directions from origin
    int min_lit = origin - (intensity / 2);
    int max_lit = origin + (intensity / 2);
    if (min_lit < 0)
    {
        min_lit = min_lit + DRAW_MAX;
    }
    if (max_lit >= DRAW_MAX)
    {
        max_lit = max_lit - DRAW_MAX;
    }
    for (int i = 0; i < DRAW_MAX; i++)
    {
        // if i is within origin +/- 1/2 intensity
        if (
            (min_lit < max_lit && min_lit < i && i < max_lit)      // range is within bounds and i is within range
            || (min_lit > max_lit && (i > min_lit || i < max_lit)) // range wraps out of bounds and i is within that wrap
        )
        {
            draw[i] = Wheel(scroll_color);
        }
        else
        {
            draw[i] = 0;
        }
    }
}

void writeSegmented()
{
    int seg_len = N_PIXELS / SEGMENTS;

    for (int s = 0; s < SEGMENTS; s++)
    {
        for (int i = 0; i < seg_len; i++)
        {
            ledsLeft[i + (s * seg_len)] = CRGB(draw[map(i, 0, seg_len, 0, DRAW_MAX)]);
        }
    }
    FastLED.show();
}

uint32_t *segmentAndResize(uint32_t *draw)
{
    int seg_len = N_PIXELS / SEGMENTS;

    uint32_t *segmented = new uint32_t(N_PIXELS);
    for (int s = 0; s < SEGMENTS; s++)
    {
        for (int i = 0; i < seg_len; i++)
        {
            segmented[i + (s * seg_len)] = draw[map(i, 0, seg_len, 0, DRAW_MAX)];
        }
    }

    return segmented;
}

void writeToStrip(uint32_t *draw)
{
    for (int i = 0; i < N_PIXELS; i++)
    {
        ledsLeft[i] = CRGB(draw[i]);
        //strip1.setPixelColor(i, draw[i]);
    }
    FastLED.show();
    //strip1.show();
}

void updateGlobals()
{
    uint16_t minLvlLeft, maxLvlLeft;
    // uint16_t minLvlRight, maxLvlRight;

    //advance color wheel
    color_wait_count++;
    if (color_wait_count > COLOR_WAIT_CYCLES)
    {
        color_wait_count = 0;
        scroll_color++;
        if (scroll_color > COLOR_MAX)
        {
            scroll_color = COLOR_MIN;
        }
    }

    volLeft[volCountLeft] = reading; // Save sample for dynamic leveling
    if (++volCountLeft >= SAMPLES)
        volCountLeft = 0; // Advance/rollover sample counter

    // Get volume range of prior frames
    minLvlLeft = maxLvlLeft = volLeft[0];
    for (uint8_t i = 1; i < SAMPLES; i++)
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
    if ((maxLvlLeft - minLvlLeft) < N_PIXELS)
        maxLvlLeft = minLvlLeft + N_PIXELS;
    minLvlAvgLeft = (minLvlAvgLeft * 63 + minLvlLeft) >> 6; // Dampen min/max levels
    maxLvlAvgLeft = (maxLvlAvgLeft * 63 + maxLvlLeft) >> 6; // (fake rolling average)
}

void vu_shatter()
{
    int intensity = calculateIntensity();
    updateOrigin(intensity);
    assignDrawValues(intensity);
    writeSegmented();
    updateGlobals();
}

#endif // __INC_MUZIO_VU_8_H