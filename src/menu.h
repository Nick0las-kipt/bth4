#ifndef _MENU_H
#define _MENU_H

#include "msm/msm_lpm13_drawer.h"


#define MENU_LINES 7
#define SCR_CHARS_IN_LINE ((DISP_W - 2*(DEF_MENU_PAD + CHAR_W))/CHAR_W)
#define MENU_CHARS_IN_LINE ((SCR_CHARS_IN_LINE > (MSM_MENU_STRLENGTH - 1)) ? (MSM_MENU_STRLENGTH - 1) : SCR_CHARS_IN_LINE)


typedef MSMtList<12> MSMList;

void menuOpen();
bool menuProcessKeys(uint8_t key, uint8_t count);
bool menuIsOpen();

void updateLoopNum();
void menuLateInitDemo();

bool shouldProlongateBl();

MSMDisplay * getMenuDisplay();

#endif