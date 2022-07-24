#ifndef DISPSERVICE_H
#define DISPSERVICE_H

#include <zephyr.h>
#include <kernel.h>


#define CHARS_PER_DISP     (DISP_W/CHAR_W)
#define CCHARS_PER_DISP    (DISP_W/CCHAR_W)
#define STATUS_BAR_Y       (MAX_LINE_DO)
#define INFO_BAR_Y         0


#define PREF_MIN_LIM 60000L
#define PREF_MAX_LIM 90000L


#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
   sunClockNormal = 0,
   sunClockCenterNoon = 1,
   sunClockCenterCurrent = 2,
   sunClockCenterUtc = 0,
} sunClockMode_e;

uint32_t getGlobalReferencePressure();
void setGlobalReferencePressure(uint32_t newPref);

bool displayStatusBar(bool force);
bool displayInfoBar(bool force);
bool displayClockTime(bool force);
bool displayClockTimeAndPressure(bool force);
bool displayLuxInfo(bool force);

#ifdef __cplusplus
}
#endif
#endif