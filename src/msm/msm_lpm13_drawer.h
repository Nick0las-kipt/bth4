#ifndef MSM_LPM13_DRAWER_H
#define MSM_LMP13_DRAWER_H

#include "msmtemplates.h"

#define MENU_DELIMETER '-'

// drawer class declared in this module
class MSMDrawer{
public:
 MSMDrawer(uint8_t x,uint8_t y,uint8_t nsym,uint8_t fcol,uint8_t bcol,uint8_t scol,uint8_t sbcol,uint8_t ncol);
 void draw(MSMDisplay* disp);
private:
  uint8_t x0;
  uint8_t y0;
  uint8_t ncl;  
  uint8_t _fcol;
  uint8_t _bcol;
  uint8_t _scol;
  uint8_t _sbcol;
  uint8_t _ncol;
};

#endif
