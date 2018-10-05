# Cine-Lights_VUs-and-More
[Youtube Channel](https://www.youtube.com/channel/UCOG6Bi2kvpDa1c8gHWZI5CQ):

A collection of Cine-Lights VU-Meters and More.
All videos and instructions are at Cine-Lights Youtube Channel.
Thanks Cine-Lights for all your Work and Effort.

# UPDATE FIX! 10/5/2018

Hi Cine-Lights I think I found the problem in the code having issues with the new versions of Arduino ide.
The problem I found was with the Juggle effect.
I changed it to the following code from: 
https://github.com/FastLED/FastLED/blob/master/examples/DemoReel100/DemoReel100.ino
The original code came I believe from:
 https://codebender.cc/sketch:91857#FastLed%20Juggle.ino
and the code works fine by itself.
The only thing that was causing the problem was the missing -1 at the end of NUM_LEDS.
and now everything works fine for me. I'm using Arduino version 1.8.5
This also goes for the "sinelon" effect added -1 as well, and I changed all instances of " LEDS."  to  "FastLED." I found in the sketch.

void juggle() {                                               // Several colored dots, weaving in and out of sync with each other
  curhue = thishue;                                          // Reset the hue values.
  fadeToBlackBy(leds, N_PIXELS, faderate);
  for ( int i = 0; i < numdots; i++) {
    //leds[beatsin16(basebeat+i+numdots,0,N_PIXELS)] += CHSV(curhue, thissat, thisbright);   //beat16 is a FastLED 3.1 function
    leds[beatsin16(basebeat+i+numdots,0,N_PIXELS - 1)] += CHSV(curhue, thissat, thisbright);   //beat16 is a FastLED 3.1 function
    //leds[beatsin16( i + 7, 0, N_PIXELS - 1 )] |= CHSV(curhue, thissat, thisbright); // <==== Original code.
    curhue += hueinc;
  }
} // juggle()

I hope this works for everybody else..
Thanks for all your hard work.
