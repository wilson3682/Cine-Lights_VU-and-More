#ifndef RGB_H_
#define RGB_H_

namespace ws2811 {
 

/**
 * Type that holds RGB values.
 * The in-memory order of this type is actually GRB, but the constructor takes
 * its values in RGB order.
 */
struct rgb
{
  rgb(uint8_t red, uint8_t green, uint8_t blue)
  :green(green),red(red),blue(blue)
  {}

  rgb()
  :green(0),red(0),blue(0)
  {}

  uint8_t green;
  uint8_t red;
  uint8_t blue;

};

}


#endif /* RGB_H_ */
