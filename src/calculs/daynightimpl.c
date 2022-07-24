#include "daynightimpl.h"
#include "rtc.h"

// day/daylight,civil,nautical, astronomical
const float dn_zenith[DAY_NIGHT_VALS] = {84.5f, 90.5f, 96.0f, 102.0f, 108.0f };

DayNightGap_t dn_gap[DAY_NIGHT_VALS];
uint8_t dn_state[DAY_NIGHT_VALS];
float dn_latitude, dn_longitude;
float civilLength, nauticalLength, astroLength;

static int dn_N = 1000;

static inline float stateToHour(uint8_t state){
    return (DayNightAlways == state) ? 25.0f : -1.0f;
}

static float getDuskLength(uint8_t pos, uint8_t ref){ // update length of dusk defined by position pos in reference with position ref
  float duskLength;
  if (dn_state[pos]){
    duskLength = stateToHour(dn_state[pos]); // we have no dusk requested
  } else {
    if (dn_state[ref]){
      // we have requested dusk but no reference requeted i.e. nautical dusk at polar night
      duskLength = fmod24(dn_gap[pos].end - dn_gap[pos].start);
    }else{
      // normal case
      duskLength = fmod24(dn_gap[pos].end - dn_gap[ref].end);
    }
  }
  return duskLength;
}

static void updateDayNightForceN(int N){
  dn_N = N;
  float dn_localoffset = 0; //Use UTC for calculations
  for(int i=0; i<DAY_NIGHT_VALS;++i){
    dn_state[i] = calculateGap(N,dn_latitude,dn_longitude,dn_localoffset, dn_zenith[i], &dn_gap[i]);
    if (dn_state[i]){
      float metaHour = stateToHour(dn_state[i]);
      dn_gap[i].start=metaHour;
      dn_gap[i].end=metaHour;
    }
  }
  civilLength = getDuskLength(2,DAY_NIGHT_RISE_N);
  nauticalLength = getDuskLength(3,DAY_NIGHT_RISE_N);
  astroLength = getDuskLength(4,DAY_NIGHT_RISE_N);
}

static int getDayNumCurrent(){
  uint8_t day, month, year;
  rtc_get_date(&day, &month, &year);
  return getDayNum(day, month, year);

}

void updateDayNightForce(){
  int N = getDayNumCurrent();
  updateDayNightForceN(N);
}

int updateDayNight(){
  int N = getDayNumCurrent();
  if (N != dn_N){
    updateDayNightForceN(N);
    return 1;
  }
  return 0;
}

int isDay(uint8_t hour, uint8_t min){
  if (DayNightAlways == dn_state[DAY_NIGHT_RISE_N]) return 1;
  if (DayNightNever == dn_state[DAY_NIGHT_RISE_N]) return 0;
  float t = (float)hour + (float)min / 60.0;
  if (dn_gap[DAY_NIGHT_RISE_N].end >= dn_gap[DAY_NIGHT_RISE_N].start){
    if ((t >= dn_gap[DAY_NIGHT_RISE_N].start) && (t <= dn_gap[DAY_NIGHT_RISE_N].end)) return 1;
  } else {
    if ((t >= dn_gap[DAY_NIGHT_RISE_N].end) && (t <= dn_gap[DAY_NIGHT_RISE_N].start)) return 1;
  }
  return 0;
}
