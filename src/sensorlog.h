#ifndef SENSORLOG_H
#define SENSORLOG_H

#include <zephyr.h>
#include <kernel.h>

#ifdef __cplusplus
extern "C" {
#endif

#define LOG_MINUTE_INTERVAL 2

uint8_t getLogRH(int pos);
int16_t getLogT(int pos);
int16_t getLogP(int pos);
uint32_t getLogPoint(int pos);
int getLogUsed();
int getLogSize();
void sensorLogFill();
void getLogPosTime(int * posToStore, uint32_t *rtcTimeToStore);
int sensorLogProcessData(int pres, int temp, int rh);


#ifdef __cplusplus
}
#endif
#endif