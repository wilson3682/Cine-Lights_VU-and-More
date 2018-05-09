#include <Adafruit_NeoPixel.h>
#include <FastLED.h> 
#include "water_torture.h"
int buttonPin = 0;    // momentary push button on pin 0
int oldButtonVal = 0;

#define PEAK_FALL 4 // Rate of peak falling dot
#define PEAK_FALL1 4 // Rate of peak falling dot
#define PEAK_FALL2 0 // Rate of peak falling dot
#define SAMPLES   100// Length of buffer for dynamic level adjustment  
#define COLOR_ORDER GRB
#define COLOR_ORDER2 BGR
#define LED_TYPE WS2812B
#define NUM_LEDS 30
#define LEFT_OUT 6
#define RIGHT_OUT 5
#define BRIGHTNESS 255
#define COLOR_START 0
#define COLOR_START2 179
#define COLOR_START3 255
#define COLOR_FROM 0
#define COLOR_TO 255
#define GRAVITY           -9.81              // Downward (negative) acceleration of gravity in m/s^2
#define h0                1                  // Starting height, in meters, of the ball (strip length)
#define NUM_BALLS         4                // Number of bouncing balls you want (recommend < 7, but 20 is fun in its own way)
#define SPEED .20       // Amount to increment RGB color by each cycle
#define BG 0
#define SPARKING 50
#define COOLING  55
#define FRAMES_PER_SECOND 60
// FOR SYLON ETC
uint8_t thisbeat =  23;
uint8_t thatbeat =  28;
uint8_t thisfade =   3;                                     // How quickly does it fade? Lower = slower fade rate.
uint8_t thissat = 255;                                     // The saturation, where 255 = brilliant colours.
uint8_t thisbri = 255; 

//FOR JUGGLE
//#define FRAMES_PER_SECOND  120
uint8_t gHue = 0; // rotating "base color" used by many of the patterns
// Twinkle
float redStates[NUM_LEDS];
float blueStates[NUM_LEDS];
float greenStates[NUM_LEDS];
float Fade = 0.96;


//config for balls
float h[NUM_BALLS] ;                         // An array of heights
float vImpact0 = sqrt( -2 * GRAVITY * h0 );  // Impact velocity of the ball when it hits the ground if "dropped" from the top of the strip
float vImpact[NUM_BALLS] ;                   // As time goes on the impact velocity will change, so make an array to store those values
float tCycle[NUM_BALLS] ;                    // The time since the last time the ball struck the ground
int   pos[NUM_BALLS] ;                       // The integer position of the dot on the strip (LED index)
long  tLast[NUM_BALLS] ;                     // The clock time of the last ground strike
float COR[NUM_BALLS] ; // Coefficient of Restitution (bounce damping)

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
int          myhue =   0;

Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, LEFT_OUT, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel strip1 = Adafruit_NeoPixel(NUM_LEDS, LEFT_OUT, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel strip2 = Adafruit_NeoPixel(NUM_LEDS, RIGHT_OUT, NEO_GRB + NEO_KHZ800);

// Water torture

WaterTorture water_torture1 = WaterTorture(&strip1);
WaterTorture water_torture2 = WaterTorture(&strip2);

// Modes
enum 
{
} MODE;
bool reverse = true;
int BRIGHTNESS_MAX = 80;
int brightness = 20;

// cycle variables
int CYCLE_MIN_MILLIS = 2;
int CYCLE_MAX_MILLIS = 1000;
int cycleMillis = 20;
bool paused = false;
long lastTime = 0;
bool boring = true;


  // Vu meter 4  
float
  greenOffset = 30,
  blueOffset = 150;  

int nPatterns = 14;
int lightPattern = 1;

const int sampleWindow = 10; // Sample window width in mS (50 mS = 20Hz)
const int sampleWindow1 = 10; // Sample window width in mS (50 mS = 20Hz)
int maximum = 110;
int maximum1 = 110;
int peak; 
int peak1;
int dotCount;
int dotCount1;
unsigned int sample;
unsigned int sample1;
bool gReverseDirection = false;

CRGB leds[NUM_LEDS];

void setup() 
{

  // edit//
   FastLED.addLeds<WS2812B, LEFT_OUT, COLOR_ORDER>(leds, NUM_LEDS).setCorrection( TypicalLEDStrip );
   FastLED.addLeds<WS2812B, RIGHT_OUT, COLOR_ORDER>(leds, NUM_LEDS).setCorrection( TypicalLEDStrip );
//   FastLED.setBrightness(  BRIGHTNESS );
   LEDS.addLeds<LED_TYPE, LEFT_OUT, COLOR_ORDER>(leds, NUM_LEDS);
   LEDS.addLeds<LED_TYPE, RIGHT_OUT, COLOR_ORDER>(leds, NUM_LEDS);
  // edit //
  
  strip1.begin();
  strip1.setBrightness(BRIGHTNESS);
  strip1.show(); // Initialize all pixels to 'off'
  strip2.begin();
  strip2.setBrightness(BRIGHTNESS);
  strip2.show(); // Initialize all pixels to 'off'


  // initialize the BUTTON pin as an input
    pinMode(buttonPin, INPUT);
    digitalWrite(buttonPin, HIGH);  // button pin is HIGH, so it drops to 0 if pressed
 
 // setup for balls//   
    
    for (int i = 0 ; i < NUM_BALLS ; i++) {    // Initialize variables
    tLast[i] = millis();
    h[i] = h0;
    pos[i] = 0;                              // Balls start on the ground
    vImpact[i] = vImpact0;                   // And "pop" up at vImpact0
    tCycle[i] = 0;
    COR[i] = 0.90 - float(i)/pow(NUM_BALLS,2); 
    }


}
// Pattern 1 - V U METER
    void VU() {

   //val = (analogRead(potPin) / 2);
   unsigned long startMillis= millis();  // Start of sample window
   unsigned int peakToPeak = 0;   // peak-to-peak level

   unsigned int signalMax = 0;
   unsigned int signalMin = 100;

   // collect data for 50 mS
   while (millis() - startMillis < sampleWindow)
   {
      sample = analogRead(A4);
      if (sample < 1024)  // toss out spurious readings
      {
         if (sample > signalMax)
         {
            signalMax = sample;  // save just the max levels
         }
         else if (sample < signalMin)
         {
            signalMin = sample;  // save just the min levels
         }
      }
   }
   peakToPeak = signalMax - signalMin;  // max - min = peak-peak amplitude

   int led = map(peakToPeak, 0, maximum, 0, strip1.numPixels()) -1;
   
   for(int i; i <= led; i++)
   {
     int color = map(i, COLOR_START, strip1.numPixels(), COLOR_FROM, COLOR_TO);
     strip1.setPixelColor(i, Wheel(color)); 
     
   }
   
   for(int i = strip1.numPixels() ; i > led; i--)
   {
     strip1.setPixelColor(i, 0); 
     
   }
  strip1.show();
 
  if(led > peak)  peak = led; // Keep 'peak' dot at top
   
   if(peak > 1 && peak <= strip1.numPixels()-1) strip1.setPixelColor(peak,Wheel(map(peak,0,strip1.numPixels()-1,0,255)));

   strip1.show();
   
// Every few frames, make the peak pixel drop by 1:

    if(++dotCount >= PEAK_FALL) { //fall rate 
      
      if(peak > 0) peak--;
      dotCount = 0;
    }
    // EDITED//
    
{
   unsigned long startMillis1= millis();  // Start of sample window
   unsigned int peakToPeak1 = 0;   // peak-to-peak level

   unsigned int signalMax1 = 0;
   unsigned int signalMin1 = 100;

   // collect data for 50 mS
   while (millis() - startMillis1 < sampleWindow1)
   {
      sample1 = analogRead(A5);
      if (sample1 < 1024)  // toss out spurious readings
      {
         if (sample1 > signalMax1)
         {
            signalMax1 = sample1;  // save just the max levels
         }
         else if (sample1 < signalMin1)
         {
            signalMin1 = sample1;  // save just the min levels
         }
      }
   }
   peakToPeak1 = signalMax1 - signalMin1;  // max - min = peak-peak amplitude
   
   int led = map(peakToPeak1, 0, maximum1, 0, strip2.numPixels()) -1;
   
   for(int i; i <= led; i++)
   {
     int color = map(i, COLOR_START, strip2.numPixels(), COLOR_FROM, COLOR_TO);
     strip2.setPixelColor(i, Wheel(color)); 
   }
   
   for(int i = strip2.numPixels() ; i > led; i--)
   {
     strip2.setPixelColor(i, 0); 
     
   }
  strip2.show();

  
  if(led > peak1)  peak1 = led; // Keep 'peak' dot at top
   
   if(peak1 > 1 && peak1 <= strip2.numPixels()-1) strip2.setPixelColor(peak1,Wheel(map(peak1,0,strip2.numPixels()-1,60,255)));

   strip2.show();
   
// Every few frames, make the peak pixel drop by 1:

    if(++dotCount1 >= PEAK_FALL1) { //fall rate 
      
     if(peak1 > 0) peak1--;
      dotCount1 = 0;
    }
}
    }

uint32_t Wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if(WheelPos < 45) {
   return strip1.Color(255 - WheelPos * 3, 0, WheelPos * 3);
   return strip2.Color(255 - WheelPos * 3, 0, WheelPos * 3);  
} else if(WheelPos < 170) {
    WheelPos -= 85;
   return strip1.Color(0, WheelPos * 3, 255 - WheelPos * 3);
   return strip2.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  } else {
   WheelPos -= 170;
   return strip1.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
   return strip2.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  }
}

void colorWipe(uint32_t c, uint8_t wait) {
  for(uint16_t i=0; i<strip1.numPixels(); i++)
  for(uint16_t i=0; i<strip2.numPixels(); i++) {
    strip1.setPixelColor(i, c);
    strip2.setPixelColor(i, c);
    strip1.show();
    strip2.show();
    delay(wait);
  }
}

void colorWipe2(uint32_t c, uint8_t wait) {
  for(uint16_t i=0; i<strip.numPixels(); i++) {
      strip1.setPixelColor(i, c);
      strip2.setPixelColor(i, c);
      strip1.show();
      strip2.show();
      delay(wait);
  }
}

// Pattern 2 - V U METER 2
    void VU2() {

 // val = (analogRead(potPin) / 2);
   unsigned long startMillis= millis();  // Start of sample window
   unsigned int peakToPeak = 0;   // peak-to-peak level

   unsigned int signalMax = 0;
   unsigned int signalMin = 100;

   // collect data for 50 mS
   while (millis() - startMillis < sampleWindow)
   {
      sample = analogRead(A4);
      if (sample < 1024)  // toss out spurious readings
      {
         if (sample > signalMax)
         {
            signalMax = sample;  // save just the max levels
         }
         else if (sample < signalMin)
         {
            signalMin = sample;  // save just the min levels
         }
      }
   }
   peakToPeak = signalMax - signalMin;  // max - min = peak-peak amplitude

   int led = map(peakToPeak, 0, maximum, 0, strip1.numPixels()) -1;
   
   for(int i; i <= led; i++)
   {
     int color = map(i, COLOR_START2, strip1.numPixels(), COLOR_FROM, COLOR_TO);
     strip1.setPixelColor(i, Wheel(color)); 
     
   }
   
   for(int i = strip1.numPixels() ; i > led; i--)
   {
     strip1.setPixelColor(i, 0); 
     
   }
  strip1.show();
//EDITED//  
  if(led > peak)  peak = led; // Keep 'peak' dot at top
   
   if(peak > 1 && peak <= strip1.numPixels()-1) strip1.setPixelColor(peak,Wheel(map(peak,0,strip1.numPixels()-1,0,255)));

   strip1.show();
   
// Every few frames, make the peak pixel drop by 1:

    if(++dotCount >= PEAK_FALL) { //fall rate 
      
      if(peak > 0) peak--;
      dotCount = 0;
    }
    // EDITED//
    
{
   unsigned long startMillis1= millis();  // Start of sample window
   unsigned int peakToPeak1 = 0;   // peak-to-peak level

   unsigned int signalMax1 = 0;
   unsigned int signalMin1 = 100;

   // collect data for 50 mS
   while (millis() - startMillis1 < sampleWindow)
   {
      sample1 = analogRead(A5);
      if (sample1 < 1024)  // toss out spurious readings
      {
         if (sample1 > signalMax1)
         {
            signalMax1 = sample1;  // save just the max levels
         }
         else if (sample1 < signalMin1)
         {
            signalMin1 = sample1;  // save just the min levels
         }
      }
   }
   peakToPeak1 = signalMax1 - signalMin1;  // max - min = peak-peak amplitude
   
   int led = map(peakToPeak1, 0, maximum1, 0, strip2.numPixels()) -1;
   
   for(int i; i <= led; i++)
   {
     int color = map(i, COLOR_START2, strip2.numPixels(), COLOR_FROM, COLOR_TO);
     strip2.setPixelColor(i, Wheel(color)); 
   }
   
   for(int i = strip2.numPixels() ; i > led; i--)
   {
     strip2.setPixelColor(i, 0); 
     
   }
  strip2.show();

  //EDITED//
  
  if(led > peak1)  peak1 = led; // Keep 'peak' dot at top
   
   if(peak1 > 1 && peak1 <= strip2.numPixels()-1) strip2.setPixelColor(peak1,Wheel(map(peak1,0,strip2.numPixels()-1,60,255)));

   strip2.show();
   
// Every few frames, make the peak pixel drop by 1:

    if(++dotCount1 >= PEAK_FALL1) { //fall rate 
      
     if(peak1 > 0) peak1--;
      dotCount1 = 0;
    }

    //EDITED//
    
}
    }


    // Pattern 2 - V U METER 2
    void VU3() {

 // val = (analogRead(potPin) / 2);
   unsigned long startMillis= millis();  // Start of sample window
   unsigned int peakToPeak = 0;   // peak-to-peak level

   unsigned int signalMax = 0;
   unsigned int signalMin = 100;

   // collect data for 50 mS
   while (millis() - startMillis < sampleWindow)
   {
      sample = analogRead(A4);
      if (sample < 1024)  // toss out spurious readings
      {
         if (sample > signalMax)
         {
            signalMax = sample;  // save just the max levels
         }
         else if (sample < signalMin)
         {
            signalMin = sample;  // save just the min levels
         }
      }
   }
   peakToPeak = signalMax - signalMin;  // max - min = peak-peak amplitude

   int led = map(peakToPeak, 0, maximum, 0, strip1.numPixels()) -1;
   
   for(int i; i <= led; i++)
   {
     int color = map(i, COLOR_START3, strip1.numPixels(), COLOR_FROM, COLOR_TO);
     strip1.setPixelColor(i, Wheel(color)); 
     
   }
   
   for(int i = strip1.numPixels() ; i > led; i--)
   {
     strip1.setPixelColor(i, 0); 
     
   }
  strip1.show();
//EDITED//  
  if(led > peak)  peak = led; // Keep 'peak' dot at top
   
   if(peak > 1 && peak <= strip1.numPixels()-1) strip1.setPixelColor(peak,Wheel(map(peak,0,strip1.numPixels()-1,0,255)));

   strip1.show();
   
// Every few frames, make the peak pixel drop by 1:

    if(++dotCount >= PEAK_FALL) { //fall rate 
      
      if(peak > 0) peak--;
      dotCount = 0;
    }
    // EDITED//
    
{
   unsigned long startMillis1= millis();  // Start of sample window
   unsigned int peakToPeak1 = 0;   // peak-to-peak level

   unsigned int signalMax1 = 0;
   unsigned int signalMin1 = 100;

   // collect data for 50 mS
   while (millis() - startMillis1 < sampleWindow)
   {
      sample1 = analogRead(A5);
      if (sample1 < 1024)  // toss out spurious readings
      {
         if (sample1 > signalMax1)
         {
            signalMax1 = sample1;  // save just the max levels
         }
         else if (sample1 < signalMin1)
         {
            signalMin1 = sample1;  // save just the min levels
         }
      }
   }
   peakToPeak1 = signalMax1 - signalMin1;  // max - min = peak-peak amplitude
   
   int led = map(peakToPeak1, 0, maximum1, 0, strip2.numPixels()) -1;
   
   for(int i; i <= led; i++)
   {
     int color = map(i, COLOR_START3, strip2.numPixels(), COLOR_FROM, COLOR_TO);
     strip2.setPixelColor(i, Wheel(color)); 
   }
   
   for(int i = strip2.numPixels() ; i > led; i--)
   {
     strip2.setPixelColor(i, 0); 
     
   }
  strip2.show();

  //EDITED//
  
  if(led > peak1)  peak1 = led; // Keep 'peak' dot at top
   
   if(peak1 > 1 && peak1 <= strip2.numPixels()-1) strip2.setPixelColor(peak1,Wheel(map(peak1,0,strip2.numPixels()-1,60,255)));

   strip2.show();
   
// Every few frames, make the peak pixel drop by 1:

    if(++dotCount1 >= PEAK_FALL1) { //fall rate 
      
     if(peak1 > 0) peak1--;
      dotCount1 = 0;
    }

    //EDITED//
    
}
    }

 //Night light 30%
    void White1() {
     
     colorWipe2(strip1.Color(50, 50, 50), 1); // white
     colorWipe2(strip2.Color(50, 50, 50), 1); // white
        
    strip1.show(); 
    strip2.show();

}   
//Night light 50%
    void White2() {
     
     colorWipe2(strip1.Color(100, 100, 100), 1); // white
     colorWipe2(strip2.Color(100, 100, 100), 1); // white
        
    strip1.show(); 
    strip2.show();

}   
//Night light 75%
    void White3() {
     
     colorWipe2(strip1.Color(150, 150, 150), 1); // white
     colorWipe2(strip2.Color(150, 150, 150), 1); // white
        
    strip1.show(); 
    strip2.show();

}   







  // Pattern 5 - black / off
    void black() {
     
     colorWipe(strip1.Color(  0, 0, 0), 1); // black
     colorWipe(strip2.Color(  0, 0, 0), 1); // black
    
      strip1.show();
      strip2.show();
    }


void Balls() {
  for (int i = 0 ; i < NUM_BALLS ; i++) {
    tCycle[i] =  millis() - tLast[i] ;     // Calculate the time since the last time the ball was on the ground

    // A little kinematics equation calculates positon as a function of time, acceleration (gravity) and intial velocity
    h[i] = 0.5 * GRAVITY * pow( tCycle[i]/1000 , 2.0 ) + vImpact[i] * tCycle[i]/1000;

    if ( h[i] < 0 ) {                      
      h[i] = 0;                            // If the ball crossed the threshold of the "ground," put it back on the ground
      vImpact[i] = COR[i] * vImpact[i] ;   // and recalculate its new upward velocity as it's old velocity * COR
      tLast[i] = millis();

      if ( vImpact[i] < 0.01 ) vImpact[i] = vImpact0;  // If the ball is barely moving, "pop" it back up at vImpact0
    }
    pos[i] = round( h[i] * (NUM_LEDS - 1) / h0);       // Map "h" to a "pos" integer index position on the LED strip
  }

  //Choose color of LEDs, then the "pos" LED on
  for (int i = 0 ; i < NUM_BALLS ; i++) leds[pos[i]] = CHSV( uint8_t (i * 40) , 255, 255);
  FastLED.show();
  //Then off for the next loop around
  for (int i = 0 ; i < NUM_BALLS ; i++) {
    leds[pos[i]] = CRGB::Black;
  }
}

//EDIT2001

// Pattern 1 - V U METER
    void VU4() {

   //val = (analogRead(potPin) / 2);
   uint8_t i;
   int n, height;
   unsigned long startMillis= millis();  // Start of sample window
   unsigned int peakToPeak = 0;   // peak-to-peak level

   unsigned int signalMax = 0;
   unsigned int signalMin = 100;

   // collect data for 50 mS
   while (millis() - startMillis < sampleWindow)
   {
      sample = analogRead(A4);
      if (sample < 1024)  // toss out spurious readings
      {
         if (sample > signalMax)
         {
            signalMax = sample;  // save just the max levels
         }
         else if (sample < signalMin)
         {
            signalMin = sample;  // save just the min levels
         }
      }
   }
   peakToPeak = signalMax - signalMin;  // max - min = peak-peak amplitude
int led = map(peakToPeak, 0, maximum, 0, strip1.numPixels()) -1;

  // Color pixels based on rainbow gradient
  for (i = 0; i < NUM_LEDS; i++) {
    if (i >= height) {
      strip2.setPixelColor(i, 0, 0, 0);
    } else {
      strip2.setPixelColor(i, Wheel(
        map(i, 0, strip2.numPixels() - 1, (int)greenOffset, (int)blueOffset)
      ));
    }
  }
  strip1.show();
 
  if(led > peak)  peak = led; // Keep 'peak' dot at top
   
   if(peak > 1 && peak <= strip1.numPixels()-1) strip1.setPixelColor(peak,Wheel(map(peak,0,strip1.numPixels()-1,0,255)));

   strip1.show();
   
// Every few frames, make the peak pixel drop by 1:

    if(++dotCount >= PEAK_FALL2) { //fall rate 
      
      if(peak > 0) peak--;
      dotCount = 0;
    }
    // EDITED//
    
{
   unsigned long startMillis1= millis();  // Start of sample window
   unsigned int peakToPeak1 = 0;   // peak-to-peak level

   unsigned int signalMax1 = 0;
   unsigned int signalMin1 = 100;

   // collect data for 50 mS
   while (millis() - startMillis1 < sampleWindow1)
   {
      sample1 = analogRead(A5);
      if (sample1 < 1024)  // toss out spurious readings
      {
         if (sample1 > signalMax1)
         {
            signalMax1 = sample1;  // save just the max levels
         }
         else if (sample1 < signalMin1)
         {
            signalMin1 = sample1;  // save just the min levels
         }
      }
   }
   peakToPeak1 = signalMax1 - signalMin1;  // max - min = peak-peak amplitude
   int led = map(peakToPeak1, 0, maximum1, 0, strip2.numPixels()) -1;

  // Color pixels based on rainbow gradient
  for (i = 0; i < NUM_LEDS; i++) {
    if (i >= height) {
      strip1.setPixelColor(i, 0, 0, 0);
    } else {
      strip1.setPixelColor(i, Wheel(
        map(i, 0, strip1.numPixels() - 1, (int)greenOffset, (int)blueOffset)
      ));
    }
  }
    strip2.show();

  
  if(led > peak1)  peak1 = led; // Keep 'peak' dot at top
   
   if(peak1 > 1 && peak1 <= strip2.numPixels()-1) strip2.setPixelColor(peak1,Wheel(map(peak1,0,strip2.numPixels()-1,60,255)));

   strip2.show();
   
// Every few frames, make the peak pixel drop by 1:

    if(++dotCount1 >= PEAK_FALL2) { //fall rate 
      
     if(peak1 > 0) peak1--;
      dotCount1 = 0;
    }
}
    }

 void ripple() {
 
    if (currentBg == nextBg) {
      nextBg = random(256);
    }
    else if (nextBg > currentBg) {
      currentBg++;
    } else {
      currentBg--;
    }
    for(uint16_t l = 0; l < NUM_LEDS; l++) {
      leds[l] = CHSV(currentBg, 255, 50);         // strip.setPixelColor(l, Wheel(currentBg, 0.1));
    }
 
  if (step == -1) {
    center = random(NUM_LEDS);
    color = random(256);
    step = 0;
  }
 
  if (step == 0) {
    leds[center] = CHSV(color, 255, 255);         // strip.setPixelColor(center, Wheel(color, 1));
    step ++;
  }
  else {
    if (step < maxSteps) {
      Serial.println(pow(fadeRate,step));
 
      leds[wrap(center + step)] = CHSV(color, 255, pow(fadeRate, step)*255);       //   strip.setPixelColor(wrap(center + step), Wheel(color, pow(fadeRate, step)));
      leds[wrap(center - step)] = CHSV(color, 255, pow(fadeRate, step)*255);       //   strip.setPixelColor(wrap(center - step), Wheel(color, pow(fadeRate, step)));
      if (step > 3) {
        leds[wrap(center + step - 3)] = CHSV(color, 255, pow(fadeRate, step - 2)*255);     //   strip.setPixelColor(wrap(center + step - 3), Wheel(color, pow(fadeRate, step - 2)));
        leds[wrap(center - step + 3)] = CHSV(color, 255, pow(fadeRate, step - 2)*255);     //   strip.setPixelColor(wrap(center - step + 3), Wheel(color, pow(fadeRate, step - 2)));
      }
      step ++;
    }
    else {
      step = -1;
    }
  }
 
  LEDS.show();
  delay(50);
}
 
 
int wrap(int step) {
  if(step < 0) return NUM_LEDS + step;
  if(step > NUM_LEDS - 1) return step - NUM_LEDS;
  return step;
}
 
 
void one_color_allHSV(int ahue, int abright) {                // SET ALL LEDS TO ONE COLOR (HSV)
  for (int i = 0 ; i < NUM_LEDS; i++ ) {
    leds[i] = CHSV(ahue, 255, abright);
  }
}

 
void ripple2() {
   if (BG){
    if (currentBg == nextBg) {
      nextBg = random(256);
    } 
    else if (nextBg > currentBg) {
      currentBg++;
    } else {
      currentBg--;
    }
    for(uint16_t l = 0; l < NUM_LEDS; l++) {
      strip1.setPixelColor(l, Wheel(currentBg, 0.1));
      strip2.setPixelColor(l, Wheel(currentBg, 0.1));
    }
  } else {
    for(uint16_t l = 0; l < NUM_LEDS; l++) {
      strip1.setPixelColor(l, 0, 0, 0);
      strip2.setPixelColor(l, 0, 0, 0);
    }
  }
 
 
  if (step == -1) {
    center = random(NUM_LEDS);
    color = random(256);
    step = 0;
  }
 
 
 
  if (step == 0) {
    strip1.setPixelColor(center, Wheel(color, 1));
    strip2.setPixelColor(center, Wheel(color, 1));
    step ++;
  } 
  else {
    if (step < maxSteps) {
      strip1.setPixelColor(wrap(center + step), Wheel(color, pow(fadeRate, step)));
      strip1.setPixelColor(wrap(center - step), Wheel(color, pow(fadeRate, step)));
      strip2.setPixelColor(wrap(center + step), Wheel(color, pow(fadeRate, step)));
      strip2.setPixelColor(wrap(center - step), Wheel(color, pow(fadeRate, step)));
      if (step > 3) {
        strip1.setPixelColor(wrap(center + step - 3), Wheel(color, pow(fadeRate, step - 2)));
        strip1.setPixelColor(wrap(center - step + 3), Wheel(color, pow(fadeRate, step - 2)));
        strip2.setPixelColor(wrap(center + step - 3), Wheel(color, pow(fadeRate, step - 2)));
        strip2.setPixelColor(wrap(center - step + 3), Wheel(color, pow(fadeRate, step - 2)));
      }
      step ++;
    } 
    else {
      step = -1;
    }
  }
  
  strip1.show();
  strip2.show();
  delay(50);
}


 
 
//int wrap(int step) {
//  if(step < 0) return NUM_LEDS + step;
//  if(step > NUM_LEDS - 1) return step - NUM_LEDS;
//  return step;
//}
 
 
//void one_color_allHSV(int ahue, int abright) {                // SET ALL LEDS TO ONE COLOR (HSV)
//  for (int i = 0 ; i < NUM_LEDS; i++ ) {
//    leds[i] = CHSV(ahue, 255, abright);
//  }
 
 
 
// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos, float opacity) {
  
  if(WheelPos < 85) {
    return strip.Color((WheelPos * 3) * opacity, (255 - WheelPos * 3) * opacity, 0);
  } 
  else if(WheelPos < 170) {
    WheelPos -= 85;
    return strip.Color((255 - WheelPos * 3) * opacity, 0, (WheelPos * 3) * opacity);
  } 
  else {
    WheelPos -= 170;
    return strip.Color(0, (WheelPos * 3) * opacity, (255 - WheelPos * 3) * opacity);
  }
}


void sinelon() {
{
  // a colored dot sweeping back and forth, with fading trails
  fadeToBlackBy( leds, NUM_LEDS, thisfade);
  int pos1 = beatsin16(thisbeat,0,NUM_LEDS);
  int pos2 = beatsin16(thatbeat,0,NUM_LEDS);
    leds[(pos1+pos2)/2] += CHSV( myhue++/64, thissat, thisbri);
}
FastLED.show();
}



void juggle() {                                               // Several colored dots, weaving in and out of sync with each other

  // eight colored dots, weaving in and out of sync with each other
  fadeToBlackBy( leds, NUM_LEDS, 20);
  byte dothue = 0;
  for( int i = 0; i < 8; i++) {
    leds[beatsin16(i+7,0,NUM_LEDS)] |= CHSV(dothue, 200, 255);
    dothue += 32;
  }
FastLED.show();

  
}



void fireblu(){
#define FRAMES_PER_SECOND 40
random16_add_entropy( random());

  
// Array of temperature readings at each simulation cell
  static byte heat[NUM_LEDS];

  // Step 1.  Cool down every cell a little
    for( int i = 0; i < NUM_LEDS; i++) {
      heat[i] = qsub8( heat[i],  random8(0, ((COOLING * 10) / NUM_LEDS) + 2));
    }
  
    // Step 2.  Heat from each cell drifts 'up' and diffuses a little
    for( int k= NUM_LEDS - 1; k >= 2; k--) {
      heat[k] = (heat[k - 1] + heat[k - 2] + heat[k - 2] ) / 3;
    }
    
    // Step 3.  Randomly ignite new 'sparks' of heat near the bottom
    if( random8() < SPARKING ) {
      int y = random8(7);
      heat[y] = qadd8( heat[y], random8(160,255) );
    }

    // Step 4.  Map from heat cells to LED colors
    for( int j = 0; j < NUM_LEDS; j++) {
      // Scale the heat value from 0-255 down to 0-240
      // for best results with color palettes.
      byte colorindex = scale8( heat[j], 240);
      CRGB color = ColorFromPalette( CRGBPalette16( CRGB::Black, CRGB::Blue, CRGB::Aqua,  CRGB::White), colorindex);
      int pixelnumber;
      if( gReverseDirection ) {
        pixelnumber = (NUM_LEDS-1) - j;
      } else {
        pixelnumber = j;
      }
      leds[pixelnumber] = color;

    }
    FastLED.show();
  }


void Drip()
{
 MODE_WATER_TORTURE: 
 if (cycle())
        {
        strip1.setBrightness(255); // off limits
        strip2.setBrightness(255); // off limits
        water_torture1.animate(reverse);
        water_torture2.animate(reverse);
        strip1.show();
        strip2.show();
       // strip1.setBrightness(brightness); // back to limited
        //strip2.setBrightness(brightness); // back to limited
        }
  }

      
bool cycle()
{
  if (paused)
  {
    return false;
  }
  
  if (millis() - lastTime >= cycleMillis)
  {
    lastTime = millis();
    return true;
  }
  return false;
}


//edit 270116
  




void Twinkle () {
   if (random(25) == 1) {
      uint16_t i = random(NUM_LEDS);
      if (redStates[i] < 1 && greenStates[i] < 1 && blueStates[i] < 1) {
        redStates[i] = random(256);
        greenStates[i] = random(256);
        blueStates[i] = random(256);
      }
    }
    
    for(uint16_t l = 0; l < NUM_LEDS; l++) {
      if (redStates[l] > 1 || greenStates[l] > 1 || blueStates[l] > 1) {
        strip1.setPixelColor(l, redStates[l], greenStates[l], blueStates[l]);
        strip2.setPixelColor(l, blueStates[l],greenStates[l], redStates[l]);
        if (redStates[l] > 1) {
          redStates[l] = redStates[l] * Fade;
        } else {
          redStates[l] = 0;
        }
        
        if (greenStates[l] > 1) {
          greenStates[l] = greenStates[l] * Fade;
        } else {
          greenStates[l] = 0;
        }
        
        if (blueStates[l] > 1) {
          blueStates[l] = blueStates[l] * Fade;
        } else {
          blueStates[l] = 0;
        }
        
      } else {
        strip1.setPixelColor(l, 0, 0, 0);
        strip2.setPixelColor(l, 0, 0, 0);
      }
    }
    strip1.show();
    strip2.show();
     delay(10);
  
}   




// the loop routine runs over and over again forever;
void loop() {
  // read that state of the pushbutton value;
  int buttonVal = digitalRead(buttonPin);
  if (buttonVal == LOW && oldButtonVal == HIGH) {// button has just been pressed
    lightPattern = lightPattern + 1;
  }
  if (lightPattern > nPatterns) lightPattern = 1;
  oldButtonVal = buttonVal;
  
  switch(lightPattern) {

    case 1:
      VU();
      break;
    case 2:
      VU2(); // синий
      break;
    case 3:
      VU3(); // зеленый
      break;
    case 4: // шарики
       VU4();
      break;
    case 5:
     ripple();
     break;
     case 6:
   ripple2();
      break;
     case 7:
   sinelon();
      break;
       case 8:
   juggle();
      break;
       case 9:
   Twinkle();
      break;
      case 10:
     Balls();
      break;
       case 11:
     fireblu();
      break;
       case 12:
     Drip();
      break;
      case 13:
     black();
      break;
     
  }
}

