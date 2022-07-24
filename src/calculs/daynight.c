#include "daynight.h"
#ifndef M_PI
#define M_PI 3.141592653
#endif
static const float cR2D = 180.0f/M_PI;
static const float cD2R = M_PI/180.0f;

/*const float zenith_day=90.5f;
const float zenith_daylight=84.5f;
const float zenith_civil=96.0f;
const float zenith_nautical=102.0f;
const float zenith_astronomical=108.0f;*/

int getDayNum(uint8_t day, uint8_t month, uint8_t year){
    int N1 = (275 * month / 9);
    int N2 = ((month + 9) / 12);
    int N3 = (1 + (((year & 0x3) + 2) / 3));
    return N1 - (N2 * N3) + day - 30;
}

static float toLocalTime(float H, float lngHour, float RA, float t, float localOffset){
    float T = H + RA - (0.06571f * t) - 6.622f;
    float UT = T - lngHour;
    float localT = UT + localOffset;
    if (localT > 24.0f) localT -= 24.0f;
    if (localT < 0) localT += 24.0f;
    return localT;
}

int calculateGap(int N, float latitude, float longitude, float localOffset, float zenith, DayNightGap_t * result)
{
#ifdef CONFIG_NEWLIB_LIBC
    float lngHour = longitude / 15.0f;
    float t = N + ((12.0f - lngHour) / 24.0f);
    float M = (0.9856f * t) - 3.289f;
    float Mrad = M*cD2R;
    float L = M + (1.916f * sin(Mrad)) + (0.020f * sin(2 * Mrad)) + 282.634f;
    if (L < 0) L += 360.0f;
    if (L > 360.0f) L -= 360.0f;
    float RA = cR2D * atan(0.91764f * tan(L*cD2R));
    float Lquadrant  = (floorf( L/90.0f)) * 90.0f;
    float RAquadrant = (floorf(RA/90.0f)) * 90.0f;
    RA += (Lquadrant - RAquadrant);
    RA /= 15.0f;
    float sinDec = 0.39782f * sin(L * cD2R);
    float cosDec = sqrt(1 - sinDec * sinDec); //cos(asin(sinDec))
    float latituderad = cD2R * latitude;
    float cosH = (cos(cD2R * zenith) - (sinDec * sin(latituderad))) / (cosDec * cos(latituderad));
    if (cosH >= 1.0f) return DayNightNever;
    if (cosH <= -1.0f) return DayNightAlways;
    float Hrise = (360.0f - cR2D * acos(cosH))/15.0f;
    float Hset = (cR2D * acos(cosH))/15.0f;
    if (result){
        result->start = toLocalTime(Hrise, lngHour, RA, t, localOffset);
        result->end   = toLocalTime(Hset,  lngHour, RA, t, localOffset);
    }
#else 
    result->start = 6.0f;
    result->end = 18.0f;
#endif
    return DayNightNormal;
}

float fmod24(float val){
  if (val > 24.0f)
    val -= 24.0f;
  else if (val < 0.0f) 
    val += 24.0f;
  return val;
}
