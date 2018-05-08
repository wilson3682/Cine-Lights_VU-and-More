#include <Adafruit_NeoPixel.h>
#include <FastLED.h> 
#include "water_torture.h"
#define PIN 6
#define COLOR_ORDER GRB
#define LED_TYPE WS2812B
#define NUM_LEDS 51
#define BRIGHTNESS 100
#define SPEED .20       // Amount to increment RGB color by each cycle
#define BG 0
#define SPARKING 50
#define COOLING  55
#define FRAMES_PER_SECOND 60
#define NUM_BALLS         3                 // Number of bouncing balls you want (recommend < 7, but 20 is fun in its own way)
#define GRAVITY           -9.81              // Downward (negative) acceleration of gravity in m/s^2
#define h0                1                  // Starting height, in meters, of the ball (strip length)
#define ARRAY_SIZE(A) (sizeof(A) / sizeof((A)[0]))
#define qsubd(x, b)  ((x>b)?wavebright:0)                   // Digital unsigned subtraction macro. if result <0, then => 0. Otherwise, take on fixed value.
#define qsuba(x, b)  ((x>b)?x-b:0)   
#define FRAMES_PER_SECOND 60
float h[NUM_BALLS] ;                         // An array of heights
float vImpact0 = sqrt( -2 * GRAVITY * h0 );  // Impact velocity of the ball when it hits the ground if "dropped" from the top of the strip
float vImpact[NUM_BALLS] ;                   // As time goes on the impact velocity will change, so make an array to store those values
float tCycle[NUM_BALLS] ;                    // The time since the last time the ball struck the ground
int   pos[NUM_BALLS] ;                       // The integer position of the dot on the strip (LED index)
long  tLast[NUM_BALLS] ;                     // The clock time of the last ground strike
float COR[NUM_BALLS] ;                       // Coefficient of Restitution (bounce damping)
int filter=80;
int highest;
int rotation=0;
int rotationdelay=0;

int pattern = 1;
// Analog Unsigned subtraction macro. if result <0, then => 0
int buttonPin = 0;    // momentary push button on pin 0
int oldButtonVal = 0;

// FOR SYLON ETC
uint8_t thisbeat =  23;
uint8_t thatbeat =  28;
uint8_t thisfade =   3;                                     // How quickly does it fade? Lower = slower fade rate.
uint8_t thissat = 255;                                     // The saturation, where 255 = brilliant colours.
uint8_t thisbri = 255; 

//FOR JUGGLE
uint8_t gHue = 0; // rotating "base color" used by many of the patterns
uint8_t numdots = 4;                                          // Number of dots in use.
uint8_t faderate = 2;                                         // How long should the trails be. Very low value = longer trails.
uint8_t hueinc = 16;                                          // Incremental change in hue between each dot.
uint8_t thishue = 0;                                          // Starting hue.
uint8_t curhue = 0; 
uint8_t thisbright = 255;                                     // How bright should the LED/display be.
uint8_t basebeat = 5; 
uint8_t max_bright = 255; 

// Twinkle
float redStates[NUM_LEDS];
float blueStates[NUM_LEDS];
float greenStates[NUM_LEDS];
float Fade = 0.96;

//Ripple variables
int color;
int center = 0;
int step = -1;
int maxSteps = 16;
float fadeRate = 0.80;
int diff;

int CYCLE_MIN_MILLIS = 2;
int CYCLE_MAX_MILLIS = 1000;
int cycleMillis = 20;
bool paused = false;
long lastTime = 0;
bool boring = true;
bool gReverseDirection = false;
int          myhue =   0;
 
//background color
uint32_t currentBg = random(256);
uint32_t nextBg = currentBg;
  

Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, PIN, NEO_GRB + NEO_KHZ800);


 static uint16_t dist;         // A random number for noise generator.
 uint16_t scale = 30;          // Wouldn't recommend changing this on the fly, or the animation will be really blocky.
 uint8_t maxChanges = 48;      // Value for blending between palettes.
 
 CRGBPalette16 currentPalette(CRGB::Black);
 CRGBPalette16 targetPalette(OceanColors_p);
// Water torture
WaterTorture water_torture = WaterTorture(&strip);

// Modes
enum 
{
} MODE;
bool reverse = true;
int BRIGHTNESS_MAX = 80;
int brightness = 20;
//Define Specific colors
uint32_t red = strip.Color(255, 0, 0);
uint32_t orange = strip.Color(255, 127, 0);
uint32_t yellow = strip.Color(255, 255, 0);
uint32_t green = strip.Color(0, 255, 0);
uint32_t blue = strip.Color(0, 0, 255);
uint32_t purple = strip.Color(75, 0, 130);



int spectrumAvg [7] ={ 40, 45, 55, 59, 80, 70, 230};
int boost [7] = { 0, 0, 0, 0, 200, 0, 100};

const int audioOutput = A0;  //data out of MSGEQ7
const int strobePin = 12;    //strobe on MSGEQ7
const int resetPin = 4;      //reset on MSGEQ7

int doubledSpectrumValue[13];
int avg = 0;
int raincolor =0; //starting point of rainbow 
int wait = 0;//8000; //time between modes
int spectrumValue[7]; //array to hold spectrum values
int middle = strip.numPixels() / 2;  //address of middle of strand
int length = strip.numPixels() / 7;  //splits strand up into 7 sections (7 bands on MGSGEQ7
int eqColors [7][3] = {{255, 0, 0}, {0, 255, 0}, {0, 0, 255}, {120, 120, 0}, {0, 120, 120}, {255, 0, 255}, {255, 255, 255}};
int eqColors2 [7][3] = {{255, 0, 0}, {0, 255, 0}, {0, 0, 255}, {255, 255, 255}, {102, 0, 102}, {255, 0, 127}, {255, 128, 0}};
int intensityColors [3] = {0,0,0}; 
uint32_t beginTime = 0;
uint32_t endTime = 0;
int bright = 1;    //select 1 thru 10
int filterValue = 115; // MSGEQ7 always spits out a number around 60, so this filters those out
int global_brightness = 96; // Sets global brightness, i.e. 64 is 1/4 brightness.
int waveSize = 10; // size 4 uses 9 lights. One in the middle, 4 on each side.  (time 7 for 63 LEDs total)
//pixels used is (wave size * 2)+1)*7
int maxOverflow = 3;

int nPatterns = 23;
int lightPattern = 1;

CRGB leds[NUM_LEDS];

void setup() 
{
  // edit//
   FastLED.addLeds<WS2812B, PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection( TypicalLEDStrip );
   LEDS.addLeds<LED_TYPE, PIN, COLOR_ORDER>(leds, NUM_LEDS);

   for (int i = 0 ; i < NUM_BALLS ; i++) {    // Initialize variables
    tLast[i] = millis();
    h[i] = h0;
    pos[i] = 0;                              // Balls start on the ground
    vImpact[i] = vImpact0;                   // And "pop" up at vImpact0
    tCycle[i] = 0;
    COR[i] = 0.90 - float(i)/pow(NUM_BALLS,2);  
   }

      
  strip.begin();
  strip.setBrightness(255);
  strip.show();
  pinMode(audioOutput, INPUT);
  pinMode(strobePin, OUTPUT);
  pinMode(resetPin, OUTPUT);
  analogReference(DEFAULT);
  
   digitalWrite(resetPin, LOW);
   digitalWrite(strobePin, HIGH);
   pinMode(buttonPin, INPUT);
   digitalWrite(buttonPin, HIGH);  // button pin is HIGH, so it drops to 0 if pressed
 
 }
void colorWipe(uint32_t c, uint8_t wait) {
  for(uint16_t i=0; i<strip.numPixels(); i++) {
    strip.setPixelColor(i, c);
    strip.show();
    delay(wait);
  }
}

  void VuOnea(){
  readaudio();
  int y=0;
  for(int i=0; i<7; i++){
    int intensity = map(spectrumValue[i], 0, 900, 0, 9);
    for(int j=(length*i); j<(length*(i+1)); j++){
      if(intensity > (j-(length*i))){
         //strip.setPixelColor(j, (Wheel(y*4)));
         strip.setPixelColor(y, strip.Color(0,0,255));
         y++;
      }
      else{
        strip.setPixelColor(j, strip.Color(0,0,0));
      }
    }
  }
  strip.setBrightness(70);
  strip.show();
}

void VuOneb(){
  readaudio();
  int y=0;
  for(int i=0; i<7; i++){
    int intensity = map(spectrumValue[i], 0, 900, 0, 9);
    for(int j=(length*i); j<(length*(i+1)); j++){
      if(intensity > (j-(length*i))){
         //strip.setPixelColor(j, (Wheel(y*4)));
         strip.setPixelColor(y, strip.Color(255,0,0));
         y++;
      }
      else{
        strip.setPixelColor(j, strip.Color(0,0,0));
      }
    }
  }
  strip.setBrightness(70);
  strip.show();
}

void VuOnec(){
  readaudio();
  int y=0;
  for(int i=0; i<7; i++){
    int intensity = map(spectrumValue[i], 0, 900, 0, 9);
    for(int j=(length*i); j<(length*(i+1)); j++){
      if(intensity > (j-(length*i))){
         //strip.setPixelColor(j, (Wheel(y*4)));
         strip.setPixelColor(y, strip.Color(0,255,0));
         y++;
      }
      else{
        strip.setPixelColor(j, strip.Color(0,0,0));
      }
    }
  }
  strip.setBrightness(70);
  strip.show();
}

void VuOne(){
  readaudio();
  int y=0;
  for(int i=0; i<7; i++){
    int intensity = map(spectrumValue[i], 0, 900, 0, 9);
    for(int j=(length*i); j<(length*(i+1)); j++){
      if(intensity > (j-(length*i))){
         //strip.setPixelColor(j, (Wheel(y*4)));
         strip.setPixelColor(y, (Wheel(y*4)));
         y++;
      }
      else{
        strip.setPixelColor(j, strip.Color(0,0,0));
      }
    }
  }
  strip.setBrightness(70);
  strip.show();
}

void VuOned(){
  readaudio();
  int y=0;
  for(int i=0; i<7; i++){
    int intensity = map(spectrumValue[i], 0, 900, 0, 9);
    for(int j=(length*i); j<(length*(i+1)); j++){
      if(intensity > (j-(length*i))){
         //strip.setPixelColor(j, (Wheel(y*4)));
         strip.setPixelColor(y, (Wheel(y*2)));
         y++;
      }
      else{
        strip.setPixelColor(j, strip.Color(0,0,0));
      }
    }
  }
  strip.setBrightness(70);
  strip.show();
}



void VuThree(){
  readaudio();
  int intensity[7];
  int total;
  int y=0;
  for(int i=0; i<7; i++){
    intensity[i] = map(spectrumValue[i], 0, 900, 0, 9);
    total+= intensity[i];
  }
  for(int m=0; m<7; m++){
    for(int p=0; p<intensity[m]; p++){
      strip.setPixelColor(y, strip.Color(eqColors2[m][0], eqColors2[m][1], eqColors2[m][2]));
      //strip.setPixelColor(y, (Wheel(y*4)));
      y++;
    }
  }
  for(int k=total; k<strip.numPixels(); k++){
    strip.setPixelColor(k, strip.Color(0,0,0));
  }
  strip.setBrightness(100);
  strip.show();
  for(int j=0; j<strip.numPixels(); j++){
    strip.setPixelColor(j, strip.Color(0,0,0));
  }
  delay(30);
}

void readaudio(){
  digitalWrite(resetPin, HIGH);
  digitalWrite(resetPin, LOW);
  
  for(int i=0; i<7; i++){
    digitalWrite(strobePin, LOW);
    delayMicroseconds(30); //allow output levels to settle
    spectrumValue[i]=analogRead(audioOutput) - spectrumAvg[i];
    digitalWrite(strobePin, HIGH);
  }
}
uint32_t Wheel(byte WheelPos) {
  if(WheelPos < 85) {
   return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  } else if(WheelPos < 170) {
   WheelPos -= 85;
   return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  } else {
   WheelPos -= 170;
   return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
}


void VuTwo()
{
  digitalWrite(resetPin, HIGH);
  digitalWrite(resetPin, LOW);
 
  //Measure the magnitudes of the 7 frequency bins
  avg = 0;
  for (int i = 0; i < 7; i++){
    digitalWrite(strobePin, LOW);
    delayMicroseconds(30); // to allow the output to settle
    spectrumValue[i] = analogRead(audioOutput);
    avg += spectrumValue[i];
    digitalWrite(strobePin, HIGH);
  }
  avg = avg/7;

  //Clear out the NeoPixel String
  for(int i = 0; i < 60; i++){
    strip.setPixelColor(i, strip.Color(0, 0, 0));
  }
  
  //Draw the meter on the NeoPixel string
  for(int i = 0; i < map(avg, 0, 1023, 0, 60); i++){
    strip.setPixelColor(i, strip.Color(i*4, 60 - i, map(spectrumValue[0], 0, 1023, 0, 60))); //Added blue flash for bass hit
    //strip.setPixelColor(i, strip.Color(i*4, 60 - i, 0)); //Without blue flash
  }
  
  strip.setBrightness(225);
  strip.show();
}


void VuFour(){
 
  if(pattern ==0){
      colorLoopProgression();
  }else if(pattern == 1){
      RainbowEqualizer();
  }
}

void RainbowEqualizer(){
  digitalWrite(resetPin, HIGH);
  digitalWrite(resetPin, LOW);

  int spectrumValue[5];
  for (int i = 0; i < 5; i++)
  {
   digitalWrite(strobePin, LOW);
   delayMicroseconds(30); // to allow the output to settle
   spectrumValue[i] = map(analogRead(audioOutput), 0, 1023, 0, 9);
   setPixel(i*10, i*10+spectrumValue[i]);
   setReverse(i*10, i*10+spectrumValue[i]);
   strip.show();
   digitalWrite(strobePin, HIGH);
  }
  delay(30);
}

  void colorLoopProgression(){
    int spectrumValue[7];
    digitalWrite(resetPin, HIGH);
    digitalWrite(resetPin, LOW);
    for (int i=0;i<7;i++){
      digitalWrite(strobePin, LOW);
      delay(10);
      spectrumValue[i]=analogRead(audioOutput);
      spectrumValue[i]=constrain(spectrumValue[i], filter, 1023);
      spectrumValue[i]=map(spectrumValue[i], filter,1023,0,255);
      //Serial.print(spectrumValue[i]);
      //Serial.print(" ");
      digitalWrite(strobePin, HIGH);
    }
    //Serial.println();

    highest =0;
    for(int i=0; i<7; i++){
      if(spectrumValue[i]>spectrumValue[highest]){
        highest = i;
      }
    }
   // Serial.println(highest);
    for(int i=30; i<60; i++){
      if(i<30+(spectrumValue[highest]/9)){
        int highestt=0;
        if(highestt==0){
          strip.setPixelColor(i, Wheel(rotation+(spectrumValue[highest]/6)+(highest*8)));
        }
        else if(highest==1){
          strip.setPixelColor(i, strip.Color(spectrumValue[highest], (255-spectrumValue[highest])/2, 0));
        }
        else if(highest==2){
          strip.setPixelColor(i, strip.Color(255-spectrumValue[highest], spectrumValue[highest], 0));
        }
        else if(highest==3){
          strip.setPixelColor(i, strip.Color(0, spectrumValue[highest], 0));
        }
        else if(highest==4){
          strip.setPixelColor(i, strip.Color(0, 0, spectrumValue[highest]));
        }
        else if(highest==5){
          strip.setPixelColor(i, strip.Color(75, 0, 130));
               }

      }
      else{
        strip.setPixelColor(i, strip.Color(0,0,0));
      }
    }
    for(int i=30; i>0; i--){
      if(i>30-(spectrumValue[highest]/9)){
        int highestt=0;
        if(highestt==0){
          strip.setPixelColor(i, Wheel(rotation+(spectrumValue[highest]/6)+(highest*8)));
        }
        else if(highest==1){
          strip.setPixelColor(i, strip.Color(255, 127, 0));
        }
        else if(highest==2){
          strip.setPixelColor(i, strip.Color(255, 255, 0));
        }
        else if(highest==3){
          strip.setPixelColor(i, strip.Color(0, 255, 0));
        }
        else if(highest==4){
          strip.setPixelColor(i, strip.Color(0, 0, 255));
        }
        else if(highest==5){
          strip.setPixelColor(i, strip.Color(75, 0, 130));
               }
      }
      else{
        strip.setPixelColor(i, strip.Color(0,0,0));
      }
    }
    if(rotationdelay>=1){
      rotation++;
      rotationdelay=0;
    }
    rotationdelay++;
    strip.show();
  }

  void setPixel(int start_bit, int pixelValue){
    for(int j=start_bit; j<pixelValue; j++){
      strip.setPixelColor(j, colorChoose(start_bit));
    }
  }


  void setReverse(int start_bit, int pixelValue){
    for(int k=start_bit+9; k>pixelValue;k--){
      strip.setPixelColor(k, 0,0,0);
    }
  }

  uint32_t colorChoose(int start_bit){
    switch (start_bit){
      case 0:
        return red;
        break;
      case 10:
        return orange;
        break;
      case 20:
        return yellow;
        break;
      case 30:
        return green;
        break;
      case 40:
        return blue;
        break;
      
    }
 
}
void VuFive() {  
     
   sevenWaves();
  
  } 

void sevenWaves(){

  int i;
  int k;
  int tmpVal;
  int r;
  int g;
  int b;
  int j;
  int x;
  int q;
  int middlePin;
  digitalWrite(resetPin, HIGH);
  digitalWrite(resetPin, LOW);
  //int overflow[77];
  
  //get readings from chip
  for (i = 0; i < 7; i++) 
  {
    
    digitalWrite(strobePin, LOW);
    //delayMicroseconds(30); // to allow the output to settle
    spectrumValue[i] = analogRead(audioOutput);
    
   
    if (i==0)
      middlePin = waveSize +1;
    else
      middlePin =  waveSize + 1 + (  (1 + waveSize * 2) * i);
    
    for (k = (middlePin - waveSize); k <= (middlePin + waveSize); k++)
    {
      int pinOutValue = 0;   
      int reading;
 
       reading = round( spectrumValue[i]*5 / (600) );
        
      int placeInWave = abs(k - middlePin);
    
      if( reading >= placeInWave)
        pinOutValue = 1;
        
      if( spectrumValue[i] < filterValue)
        pinOutValue = 0;
      
           
     if(pinOutValue==1)
       pinOutValue = 255;
      
      // RGB for bands 1-7
      if(i==0)
      {
        r = 0; g = 0; b = 255;
      }
      if(i==1)
      {
        r = 0; g = 255; b= 0;
      }
      if(i==2)
      {
        r = 255; g = 255; b = 0;
      }
      if(i==3)
      {
        r = 0; g = 255; b = 0;
      }
      if(i==4)
      {
        r = 0; g = 255; b = 255;
      }
      if(i==5)
      {
        r = 0; g = 0 ; b = 255;
      }
      if(i==6)
      {
        r = 255; g = 0; b = 153;
      }
      
      if (pinOutValue == 0)
      {
        r = 0; g = 0; b = 0;
      }
      strip.setPixelColor(k-1, r, g, b); 
    
    }    
    digitalWrite(strobePin, HIGH);
  }
  
   
  strip.setBrightness(global_brightness);
  strip.show();
  
}

void sevenBands(){
  int i;
  digitalWrite(resetPin, HIGH);
  digitalWrite(resetPin, LOW);

//grabs readings from chip
  for (i = 0; i < 7; i++)
  {
    digitalWrite(strobePin, LOW);
   // delayMicroseconds(30); // to allow the output to settle
    spectrumValue[i] = analogRead(audioOutput);
 
 
   
  if(spectrumValue[i] < filterValue)
     spectrumValue[i] = 0;
 
 spectrumValue[i] = spectrumValue[i] / (1024/255);
 
 if(spectrumValue[i] > 255)
     spectrumValue[i] = 255;

    
     digitalWrite(strobePin, HIGH);
  }

  strip.setPixelColor(0, spectrumValue[6], 0, 0);
  strip.setPixelColor(1, spectrumValue[5], 0, 0);
  strip.setPixelColor(2, spectrumValue[4], 0, 0);
  strip.setPixelColor(3, spectrumValue[3], 0, 0);
  strip.setPixelColor(4, spectrumValue[2], 0, 0);
  strip.setPixelColor(5, spectrumValue[1], 0, 0);
  strip.setPixelColor(6, spectrumValue[0], 0, 0);

  strip.setBrightness(global_brightness);
  strip.show();
}

void nextLed(int r, int g, int b, int wait){
static int currentLed = 0;


strip.setPixelColor(currentLed, r, g, b);
delay(wait);

//if (currentLed % 4 == 0)
 // strip.show();

currentLed++;

if (currentLed >= strip.numPixels() ){
  currentLed = 0;
  strip.show();
}
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



// Slightly different, this makes the rainbow equally distributed throughout
void rainbowCycle(uint8_t wait) {
  uint16_t i, j;

  for(j=0; j<256*5; j++) { // 5 cycles of all colors on wheel
    for(i=0; i< strip.numPixels(); i++) {
      strip.setPixelColor(i, Wheel(((i * 256 / strip.numPixels()) + j) & 255));
    }
    strip.show();
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
      strip.setPixelColor(l, Wheel(currentBg, 0.1));
    }
  } else {
    for(uint16_t l = 0; l < NUM_LEDS; l++) {
      strip.setPixelColor(l, 0, 0, 0);
    }
  }
 
 
  if (step == -1) {
    center = random(NUM_LEDS);
    color = random(256);
    step = 0;
  }
 
 
 
  if (step == 0) {
    strip.setPixelColor(center, Wheel(color, 1));
    step ++;
  } 
  else {
    if (step < maxSteps) {
      strip.setPixelColor(wrap(center + step), Wheel(color, pow(fadeRate, step)));
      strip.setPixelColor(wrap(center - step), Wheel(color, pow(fadeRate, step)));
      if (step > 3) {
        strip.setPixelColor(wrap(center + step - 3), Wheel(color, pow(fadeRate, step - 2)));
        strip.setPixelColor(wrap(center - step + 3), Wheel(color, pow(fadeRate, step - 2)));
      }
      step ++;
    } 
    else {
      step = -1;
    }
  }
  
  strip.show();
  delay(50);
}

void fire(){
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
      CRGB color = ColorFromPalette( CRGBPalette16( CRGB::Black, CRGB::Red, CRGB::Yellow,  CRGB::White), colorindex);
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
        strip.setBrightness(255); // off limits
        water_torture.animate(reverse);
        strip.show();
        //strip.setBrightness(brightness); // back to limited
        
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


   void pattern2() {
      
       sinelon();                                                  // Call our sequence.
  show_at_max_brightness_for_power();                         // Power managed display of LED's.
} // loop()


void sinelon() {
  // a colored dot sweeping back and forth, with fading trails
  fadeToBlackBy( leds, NUM_LEDS, thisfade);
  int pos1 = beatsin16(thisbeat,0,NUM_LEDS);
  int pos2 = beatsin16(thatbeat,0,NUM_LEDS);
    leds[(pos1+pos2)/2] += CHSV( myhue++/64, thissat, thisbri);
}
// Pattern 3 - JUGGLE
    void pattern3() {
       ChangeMe();
  juggle();
  show_at_max_brightness_for_power();                         // Power managed display of LED's.
} // loop()


void juggle() {                                               // Several colored dots, weaving in and out of sync with each other
  curhue = thishue;                                          // Reset the hue values.
  fadeToBlackBy(leds, NUM_LEDS, faderate);
  for( int i = 0; i < numdots; i++) {
    leds[beatsin16(basebeat+i+numdots,0,NUM_LEDS)] += CHSV(curhue, thissat, thisbright);   //beat16 is a FastLED 3.1 function
    curhue += hueinc;
  }
} // juggle()


void ChangeMe() {                                             // A time (rather than loop) based demo sequencer. This gives us full control over the length of each sequence.
  uint8_t secondHand = (millis() / 1000) % 30;                // IMPORTANT!!! Change '30' to a different value to change duration of the loop.
  static uint8_t lastSecond = 99;                             // Static variable, means it's only defined once. This is our 'debounce' variable.
  if (lastSecond != secondHand) {                             // Debounce to make sure we're not repeating an assignment.
    lastSecond = secondHand;
    if (secondHand ==  0)  {numdots=1; faderate=2;}  // You can change values here, one at a time , or altogether.
    if (secondHand == 10)  {numdots=4; thishue=128; faderate=8;}
    if (secondHand == 20)  {hueinc=48; thishue=random8();}                               // Only gets called once, and not continuously for the next several seconds. Therefore, no rainbows.
  }
} // ChangeMe()

void juggle2() {                                               // Several colored dots, weaving in and out of sync with each other

  // eight colored dots, weaving in and out of sync with each other
  fadeToBlackBy( leds, NUM_LEDS, 20);
  byte dothue = 0;
  for( int i = 0; i < 8; i++) {
    leds[beatsin16(i+7,0,NUM_LEDS)] |= CHSV(dothue, 200, 255);
    dothue += 32;
  }
FastLED.show();

  
}



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
        strip.setPixelColor(l, redStates[l], greenStates[l], blueStates[l]);
        
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
        strip.setPixelColor(l, 0, 0, 0);
      }
    }
    strip.show();
     delay(10);
  
}

void blur() {

  uint8_t blurAmount = dim8_raw( beatsin8(3,64, 192) );       // A sinewave at 3 Hz with values ranging from 64 to 192.
  blur1d( leds, NUM_LEDS, blurAmount);                        // Apply some blurring to whatever's already on the strip, which will eventually go black.
  
  uint8_t  i = beatsin8(  9, 0, NUM_LEDS);
  uint8_t  j = beatsin8( 7, 0, NUM_LEDS);
  uint8_t  k = beatsin8(  5, 0, NUM_LEDS);
  
  // The color of each point shifts over time, each at a different speed.
  uint16_t ms = millis();  
  leds[(i+j)/2] = CHSV( ms / 29, 200, 255);
  leds[(j+k)/2] = CHSV( ms / 41, 200, 255);
  leds[(k+i)/2] = CHSV( ms / 73, 200, 255);
  leds[(k+i+j)/3] = CHSV( ms / 53, 200, 255);
  
  FastLED.show();
  
} // loop()


 

void rainbow(uint8_t wait) {
  uint16_t i, j;

  for(j=0; j<256; j++) {
    for(i=0; i<strip.numPixels(); i++) {
      strip.setPixelColor(i, Wheel((i+j) & 255));
    }
    strip.show();
    }
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
     All2();   
       break;
       case 2:
     All();   
       break;
    case 3:
     VuOnea();   
       break;
       case 4:
     VuOneb();   
       break;
        case 5:
     VuOnec();   
       break;
          case 6:
     VuOned();   
       break;
       case 7:
     VuOne();   
       break;
       case 8:
      VuTwo();
       break;
      case 9:
      VuThree();
     break;
     case 10:
      VuFour();
      break;
    case 11:
     VuFive();
      break;
    case 12: 
      ripple();
      break;
    case 13:
      ripple2();
     break;
     case 14:
   Twinkle();
     break;
     case 15:
    pattern2(); // sylon
     break;
     case 16:
     pattern3();
      break;
     case 17:
    juggle2();
      break;
       case 18:
   blur();
      break;
      case 19:
      Balls(); // 
      break;
      case 20:
    Drip(); // 
      break;
       case 21:
    fireblu();
      break;
      case 22:
       fire(); // 
      break;

      case 23:
       colorWipe(strip.Color(0, 0, 0), 10); // Black
      break;
     
  }
}

// List of patterns to cycle through.  Each is defined as a separate function below.
typedef void (*SimplePatternList[])();
SimplePatternList gPatterns = {ripple, ripple2, Twinkle, pattern2, juggle2, pattern3, blur, Balls, Drip, fireblu, fire};
uint8_t gCurrentPatternNumber = 0; // Index number of which pattern is current

void nextPattern()
{
  // add one to the current pattern number, and wrap around at the end
  gCurrentPatternNumber = (gCurrentPatternNumber + 1) % ARRAY_SIZE( gPatterns);
}
void All()
{
  // Call the current pattern function once, updating the 'leds' array
  gPatterns[gCurrentPatternNumber]();
  EVERY_N_SECONDS( 30 ) { nextPattern(); } // change patterns periodically
}
// second list

// List of patterns to cycle through.  Each is defined as a separate function below.
typedef void (*SimplePatternList[])();
SimplePatternList qPatterns = {VuOnea,  VuOneb, VuOnec, VuOned, VuOne, VuTwo, VuThree, VuFour, VuFive};
uint8_t qCurrentPatternNumber = 0; // Index number of which pattern is current

void nextPattern2()
{
  // add one to the current pattern number, and wrap around at the end
  qCurrentPatternNumber = (qCurrentPatternNumber + 1) % ARRAY_SIZE( qPatterns);
}
void All2()
{
  // Call the current pattern function once, updating the 'leds' array
  qPatterns[qCurrentPatternNumber]();
  EVERY_N_SECONDS( 30 ) { nextPattern2(); } // change patterns periodically
}



