#ifndef _METEOMATH_H
#define _METEOMATH_H

#include <inttypes.h>
#include <math.h>

#define KELVIN_0C 273.15f

#ifdef __cplusplus
extern "C" {
#endif

int16_t getDewPoint(int16_t temp, uint8_t RH);
uint8_t getRHfromDewTemp(int16_t temp, int16_t TDI);
int16_t getBaroAlt(uint32_t P, uint32_t pRef, int16_t T);
uint32_t getPressureFromAlt(int16_t alt, uint32_t pRef, int16_t T);

typedef struct {
  float x;
  float y;
} colorxy_t;

colorxy_t getUVfromXY(colorxy_t colXY);
colorxy_t getXYfromUV(colorxy_t colUV);
colorxy_t getUVfromColorTemp(float t);
float getColorTemp(colorxy_t color);

#ifdef __cplusplus
}
#endif

#endif