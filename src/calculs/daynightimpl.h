#ifndef DAYNIGHTBTH_H
#define DAYNIGHTBTH_H
#include "daynight.h"


#define DAY_NIGHT_VALS 5 
#define DAY_NIGHT_RISE_N 1 

#ifdef __cplusplus
extern "C" {
#endif

// shows when day or dusk starts and finishes
extern DayNightGap_t dn_gap[DAY_NIGHT_VALS];
// state of corresponding events (normal/never/always)
extern uint8_t dn_state[DAY_NIGHT_VALS];
// Dusk length
extern float civilLength, nauticalLength, astroLength;
// Location settings
extern float dn_latitude, dn_longitude;

void updateDayNightForce();
int updateDayNight();
int isDay(uint8_t hour, uint8_t min);

#ifdef __cplusplus
};
#endif


#endif
