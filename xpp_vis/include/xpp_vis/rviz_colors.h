
#ifndef XPP_VIS_RVIZ_COLORS_H_
#define XPP_VIS_RVIZ_COLORS_H_

#include <std_msgs/ColorRGBA.h>

namespace xpp {

/**
 * @brief Supplies some predefined colors to be used for RVIZ markers.
 *
 * A static object "color" of this is creates, so in order to use the colors,
 * type e.g. color.red;
 */
struct ColorT {

  ColorT() {
    // set alpha for all
    red.a = green.a = blue.a = white.a = brown.a
    = yellow.a = purple.a = black.a = gray.a = wheat.a = 1.0;

    black.r  =           black.g  =           black.b  = 0.1;
    gray.r   =           gray.g   =           gray.b   = 0.7;
    white.b  =           white.g  =           white.r  = 1.0;
    red.r    = 1.0;      red.g    = 0.0;      red.b    = 0.0;
    green.r  = 0.0;      green.g  = 150./255; green.b  = 76./255;
    blue.r   = 0.0;      blue.g   = 102./255; blue.b   = 204./255;
    dblue.r  = 0.0;      dblue.g  = 0./255;   dblue.b  = 128./255;
    brown.r  = 122./255; brown.g  = 61./255;  brown.b  = 0.0;
    yellow.r = 204./255; yellow.g = 204./255; yellow.b = 0.0;
    purple.r = 72./255;  purple.g = 61./255;  purple.b = 139./255;
    wheat.r  = 245./355; wheat.g  = 222./355; wheat.b  = 179./355;
  };

  virtual ~ColorT() {};

  std_msgs::ColorRGBA red, green, blue, dblue, white, brown,
                      yellow, purple, black, gray, wheat;
};

static ColorT color; // create instance for efficient access

} /* namespace xpp */

#endif /* XPP_VIS_RVIZ_COLORS_H_ */
