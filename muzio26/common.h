#ifndef __INC_MUZIO_COMMON_H
#define __INC_MUZIO_COMMON_H
#include <FastLED.h>

#define N_PIXELS 26 // Number of pixels in strand
#define N_PIXELS_HALF (N_PIXELS / 2)
#define MIC_PIN A5          // Microphone is attached to this analog pin
#define LED_PIN 6           // NeoPixel LED strand is connected to this pin
#define SAMPLE_WINDOW 10    // Sample window for average level
#define SAMPLE_WINDOW_2 10  // Sample window for average level
#define PEAK_FALL3 50       //Rate of falling peak dot
#define PEAK_HANG 30        //Time of pause before peak dot falls
#define PEAK_HANG2 40       //Time of pause before peak dot falls
#define PEAK_FALL 30        //Rate of falling peak dot
#define PEAK_FALL2 8        //Rate of falling peak dot
#define INPUT_FLOOR 10      //Lower range of analogRead input
#define INPUT_CEILING 300   //Max range of analogRead input, the lower the value the more sensitive (1023 = max)300 (150)
#define DC_OFFSET 0         // DC offset in mic signal - if unusure, leave 0
#define NOISE 10            // Noise/hum/interference in mic signal
#define SAMPLES 64          // Length of buffer for dynamic level adjustment
#define SAMPLES2 64         // Length of buffer for dynamic level adjustment
#define TOP (N_PIXELS + 2)  // Allow dot to go slightly off scale
#define SPEED .20           // Amount to increment RGB color by each cycle
#define TOP2 (N_PIXELS + 1) // Allow dot to go slightly off scale
#define LAST_PIXEL_OFFSET N_PIXELS - 1
#define PEAK_FALL_MILLIS 10 // Rate of peak falling dot
#define FRAMES_PER_SECOND 180
#define POT_PIN 4
#define BG 0

#define BRIGHTNESS 255
#define LED_TYPE WS2812B // Only use the LED_PIN for WS2812's
#define COLOR_ORDER GRB
#define COLOR_MIN 0
#define COLOR_MAX 255
#define DRAW_MAX 100
#define SEGMENTS 4           // Number of segments to carve amplitude bar into
#define COLOR_WAIT_CYCLES 10 // Loop cycles to wait between advancing pixel origin
#define qsubd(x, b) ((x > b) ? b : 0)
#define qsuba(x, b) ((x > b) ? x - b : 0) // Analog Unsigned subtraction macro. if result <0, then => 0. By Andrew Tuline.
#define ARRAY_SIZE(A) (sizeof(A) / sizeof((A)[0]))

#define MUZIO_DEBUG 1     // 1 to DEBUG, 0 to disable

uint8_t volCountLeft = 0; // Frame counter for storing past volume data
int lvlLeft = 10;         // Current "dampened" audio level
int lvlLeft2 = 10;        // Current "dampened" audio level
int minLvlAvgLeft = 0;    // For dynamic adjustment of graph low & high
int maxLvlAvgLeft = 512;

uint8_t volCountRight = 0; // Frame counter for storing past volume data
int volRight[SAMPLES];     // Collection of prior volume samples
int lvlRight = 10;         // Current "dampened" audio level
int minLvlAvgRight = 0;    // For dynamic adjustment of graph low & high
int maxLvlAvgRight = 512;

byte
    peakLeft = 0, // Used for falling dot
    peakRight = 0,
    dotCountLeft = 0,  // Frame counter for delaying dot-falling speed
    dotCountRight = 0, // Frame counter for delaying dot-falling speed
    volCount = 0;      // Frame counter for storing past volume data

int reading,
    vol[SAMPLES],  // Collection of prior volume samples
    lvl = 10,      // Current "dampened" audio level
    minLvlAvg = 0, // For dynamic adjustment of graph low & high
    maxLvlAvg = 512;

CRGB ledsLeft[N_PIXELS];

//Variables will change:
uint8_t buttonPushCounter = 0; // counter for the number of button presses
int buttonState = 0;           // current state of the button
int lastButtonState = 0;

byte peak = 16;        // Peak level of column; used for falling dots
byte dotCount = 0;     //Frame counter for peak dot
byte dotHangCount = 0; //Frame counter for holding peak dot

// Arduino , maximum is 1023
uint16_t volLeft[SAMPLES];     // Collection of prior volume samples

// cycle variables
int CYCLE_MIN_MILLIS = 2;
int CYCLE_MAX_MILLIS = 1000;
int cycleMillis = 20;
bool paused = false;
bool boring = true;
int myhue = 0;

////vu ripple
uint8_t colour;
uint8_t myfade = 255; // Starting brightness.
#define maxsteps 16   // Case statement wouldn't allow a variable.
int peakspersec = 0;
int peakcount = 0;
uint8_t bgcol = 0;
int thisdelay = 20;
uint8_t max_bright = 255;

unsigned int sample;

//Samples
#define NSAMPLES 64
unsigned int samplearray[NSAMPLES];
unsigned long samplesum = 0;
unsigned int sampleavg = 0;
int samplecount = 0;
unsigned long oldtime = 0;
unsigned long newtime = 0;

//Ripple variables
int color;
int center = 0;
int step = -1;
int maxSteps = 16;
float fadeRate = 0.80;
int diff;

//background color
uint32_t currentBg = random(256);
uint32_t nextBg = currentBg;
TBlendType    currentBlending;  

uint32_t colorToInt(uint8_t r, uint8_t g, uint8_t b);
uint32_t red = colorToInt(255, 0, 0);
// uint32_t orange = colorToInt(255, 127, 0);
// uint32_t yellow = colorToInt(255, 255, 0);
uint32_t green = colorToInt(0, 255, 0);
// uint32_t blue = colorToInt(0, 0, 255);
// uint32_t purple = colorToInt(75, 0, 130);
// uint32_t white = colorToInt(125, 125, 125);

#endif // __INC_MUZIO_COMMON_H