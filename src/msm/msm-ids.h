// Menu Structure Module main module
// Defenition of MSM classes

#ifndef __MSM_IDS_H_
#define __MSM_IDS_H_ 1

#ifdef __cplusplus
extern "C" {
#endif

// Keys
#define MSM_LIM_COUNT   10
#define MSM_BRK_COUNT   5

#define MSM_KEY_ESC     1
#define MSM_KEY_ENTER   2
#define MSM_KEY_LEFT    3
#define MSM_KEY_RIGHT   4
#define MSM_KEY_UP      5
#define MSM_KEY_DOWN    6

// Flags
#define MSM_FL_RAMNAME       8
#define MSM_FL_UPDATED      16
#define MSM_FL_LIVEUPDATE   32
#define MSM_FL_MM         0xF8

#define MSM_MODE_NORMAL      0
#define MSM_MODE_CLOSELEVEL  1
#define MSM_MODE_CLOSEALL    2
#define MSM_MODE_SUBMENU     3
#define MSM_MODE_OP          4
#define MSM_MODE_MM       0x07

#define LISTCALSE_AFTER      1
#define LISTCALSE_BEFORE     0

#ifdef __cplusplus
}
#endif
#endif
