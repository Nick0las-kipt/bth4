#ifndef __DAYNIGHT_H
#define __DAYNIGHT_H
#include <math.h>
#include <inttypes.h>

#ifdef __cplusplus
extern "C" {
#endif


typedef enum {
    DayNightNormal  = 0,
    DayNightAlways  = 1,
    DayNightNever   = 2,
}
DayNightMode_e;

typedef struct {
    float start;
    float end;
} DayNightGap_t;

int getDayNum(uint8_t day, uint8_t month, uint8_t year);
int calculateGap(int N, float latitude, float longitude, float localOffset, float zenith, DayNightGap_t * result);
float fmod24(float val);

extern const float zenith_day;
extern const float zenith_daylight;
extern const float zenith_civil;
extern const float zenith_nautical;
extern const float zenith_astronomical;

#ifdef __cplusplus
};
#endif

#endif
