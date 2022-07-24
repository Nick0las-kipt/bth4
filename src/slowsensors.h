#ifndef SLOWSENSORS_H
#define SLOWSENSORS_H

#include <zephyr.h>
#include <kernel.h>

#define SENSORS_SLEEP_TIME_MS             8000
#define SENSORS_SLEEP_TIME_SHORT_MS        300

#ifdef __cplusplus
extern "C" {
#endif

void slowsensors_init(void);
k_tid_t slowsensors_get_tid(void);

static inline int32_t pascals2torr(int32_t pressureInPascals){
  return ((pressureInPascals *12289)>>14);
}

int getBattPercentage();

#define FLAG_SENSOR_TEMPERATURE_BAROSENSOR  BIT(0)
int32_t slowsensorsGetTemperature(uint32_t flags);

#define FLAG_SENSOR_PRESSURE_SMOOTH         BIT(0)
#define FLAG_SENSOR_PRESSURE_MMHG           BIT(1)

#define FLAG_SENSOR_RH_SHIFT8               BIT(0)

int32_t slowsensorsGetPressure(uint32_t flags);

uint32_t slowsensorsGetHumidity(uint32_t flags);

#define FLAG_SENSOR_LUX_RAW_NW              BIT(8)
#define FLAG_SENSOR_LUX_CHANNEL_MASK        (BIT(8) - 1)

uint32_t slowsensorsGetLux(uint32_t flags, uint32_t multiplyer);
int slowsensorsLuxInSync(void);
void slowsensorsGetColorXY(uint32_t *px, uint32_t *py);

void slowsensorsSetFastRun(int fast);

#ifdef __cplusplus
}
#endif

#endif