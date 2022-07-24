#ifndef RTC_H
#define RTC_H
#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
     dst_mode_none = 0,
     dst_mode_EU = 1,
     dst_mode_SU = 2,
     dst_mode_US = 3,
} dst_mode_e;

typedef struct timezone_s {
    int8_t delta_QH; //
    uint8_t dst_mode;
} timezone_t;

typedef struct tzNameVal_s {
    char str[16];
    timezone_t tz;
} tzNameVal_t;

void rtc_time_flatten();

void rtc_set_corr(int32_t delta_10m);

int rtc_init(void);

uint32_t rtc_time_secs_get(void);
uint32_t rtc_time_secs_get_tz(void);

int32_t rtc_tz_delta(uint32_t secs);

void rtc_set_tz(timezone_t timezone);
timezone_t rtc_get_tz(void);

int days_in_month(int month,int year);

void rtc_secs_to_hms(uint32_t secs, uint8_t * phour, uint8_t * pmin, uint8_t * psec);
void rtc_days_to_date(uint32_t days, uint8_t * pday, uint8_t * pmonth, uint8_t * pyear);

uint32_t rtc_hms_to_secs(int hour, int min, int sec);
uint32_t rtc_date_to_days(int day, int month, int year);

void rtc_get_hms(uint8_t * phour, uint8_t * pmin, uint8_t * psec);
void rtc_get_date(uint8_t * pday, uint8_t * pmonth, uint8_t * pyear);

void rtc_get_date_time_from_secs(uint32_t secs, uint8_t * pday, uint8_t * pmonth, uint8_t * pyear, uint8_t * phour, uint8_t * pmin, uint8_t * psec);

void _rtc_set_date_time(int day, int month, int year, int hour, int min, int sec, int tz_flag);
void _rtc_get_date_time_dow(uint8_t * pday, uint8_t * pmonth, uint8_t * pyear, uint8_t * phour, uint8_t * pmin, uint8_t * psec, uint8_t * pdow, int tz_flag);

uint8_t rtc_get_dow_from_secs(uint32_t secs);

static inline void rtc_get_date_time(uint8_t * pday, uint8_t * pmonth, uint8_t * pyear, uint8_t * phour, uint8_t * pmin, uint8_t * psec){
    return _rtc_get_date_time_dow(pday, pmonth, pyear, phour, pmin, psec, 0, 0);
}

static inline void rtc_get_date_time_tz(uint8_t * pday, uint8_t * pmonth, uint8_t * pyear, uint8_t * phour, uint8_t * pmin, uint8_t * psec){
    return _rtc_get_date_time_dow(pday, pmonth, pyear, phour, pmin, psec, 0, 1);
}

static inline void rtc_set_date_time(int day, int month, int year, int hour, int min, int sec) {
     return _rtc_set_date_time(day, month, year, hour, min, sec, 0);
}

static inline void rtc_set_date_time_tz(int day, int month, int year, int hour, int min, int sec) {
     return _rtc_set_date_time(day, month, year, hour, min, sec, 1);
}

static inline void rtc_get_date_time_dow(uint8_t * pday, uint8_t * pmonth, uint8_t * pyear, uint8_t * phour, uint8_t * pmin, uint8_t * psec, uint8_t * pdow){
    return _rtc_get_date_time_dow(pday, pmonth, pyear, phour, pmin, psec, pdow, 0);    
}

static inline void rtc_get_date_time_dow_tz(uint8_t * pday, uint8_t * pmonth, uint8_t * pyear, uint8_t * phour, uint8_t * pmin, uint8_t * psec, uint8_t * pdow){
    return _rtc_get_date_time_dow(pday, pmonth, pyear, phour, pmin, psec, pdow, 1);    
}

extern const tzNameVal_t * const tzVariants[];

int getTzIndexByTz(timezone_t tz);

#ifdef __cplusplus
}
#endif

#endif