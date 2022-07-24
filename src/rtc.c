#include <zephyr.h>
#include <kernel.h>
#include "rtc-platform.h"
#include "rtc.h"

// RTC vars
//START TIME POINT is 01 Jan 2000 0.0 UTC. All time points are after
static uint32_t secs_from_start_to_ref;  //This are secs in UTC
static int32_t  xtal_rel_pp10m;; // XTAL correction in parts per 10 million (0.1ppm)
static uint64_t ticks_at_ref;

static timezone_t timezone_i = {
    .delta_QH = 0,
    .dst_mode = dst_mode_none,
}; // Time Zone

#define SECS_IN_DAY  (86400)
#define SECS_IN_YEAR (SECS_IN_DAY*365)

static uint32_t rtc_time_secs_get_impl(uint64_t ticks){
    return secs_from_start_to_ref + ( (ticks - ticks_at_ref) * xtal_rel_pp10m / (10000000ULL * (PL_TICKS_HZ)) );
}

uint32_t rtc_time_secs_get(void){
    return rtc_time_secs_get_impl(rtc_platform_ticks_get());
}

void rtc_time_flatten(){
    uint64_t ticks = rtc_platform_ticks_get();
    uint32_t secs = rtc_time_secs_get_impl(ticks);
    secs_from_start_to_ref = secs;
    ticks_at_ref = ticks;
}

void rtc_set_corr(int32_t delta_10m){
    rtc_time_flatten();
    xtal_rel_pp10m = 10000000LL + delta_10m;
}

int rtc_init(void){
    int rv = rtc_platform_init();
    xtal_rel_pp10m = 10000000LL; //TODO:load from config
    return rv;
}

static int32_t get_tz_delta_def(timezone_t timezone_i){
    return ((int32_t)timezone_i.delta_QH)*900;
}

static int32_t get_tz_delta_dst(uint32_t secs_no_dst, timezone_t timezone_i){
    int32_t delta = 0;
    if (dst_mode_none != timezone_i.dst_mode){
        uint8_t hour;
        uint8_t day, month;
        uint8_t dow = rtc_get_dow_from_secs(secs_no_dst);
        rtc_get_date_time_from_secs(secs_no_dst, &day, &month, NULL, &hour, NULL, NULL);
        if ((dst_mode_SU == timezone_i.dst_mode) || (dst_mode_EU == timezone_i.dst_mode)){
            if ((month > 3) && (month < 10)){
                // From April to September it is ALWAYS DST
                delta = 3600;
            } else if ((3 == month) || (10 == month)){ // march or october
                // Calculate last sunday. dow = 1..7, 7 = sunday
                int days_to_sunday = 7 - dow;
                int some_sunday = day + days_to_sunday;
                int last_day = (3 == month) ? 31: 30;
                if (some_sunday > last_day) {
                    some_sunday -= 7;
                } else {
                    while ((7 + some_sunday) <= last_day) some_sunday += 7;
                }
                int last_sunday = some_sunday;
                if (day < last_sunday){
                    if (10 == month) delta = 3600;
                } else if (day > last_sunday) {
                    if (3 == month) delta = 3600;
                } else { // day = last_sunday
                    // EU strasitions ad 1am UTC, SU transitions at 2am local
                    int transitionHour = (dst_mode_EU == timezone_i.dst_mode) ? 1 + (timezone_i.delta_QH >> 2) : 2;
                    if (3 == month){
                        if (hour > transitionHour)delta = 3600;
                    } else { // month = 10
                        if (hour < transitionHour) delta = 3600;
                    }
                } // end last sunday
            } // end march or october
        } else if ((dst_mode_US == timezone_i.dst_mode)){
            if ((month > 3) && (month < 11)){
                // From April to October it is ALWAYS DST in US
                delta = 3600;
            } else if ((3 == month) || (11 == month)){ // march or november
                // Calculate last sunday. dow = 1..7, 7 = sunday
                int days_to_sunday = 7 - dow;
                int some_sunday = day + days_to_sunday;
                while ((some_sunday - 7) > 0) some_sunday -= 7;

                int transition_sunday = (3 == month) ? some_sunday + 7 : some_sunday;
                if (day < transition_sunday){
                    if (11 == month) delta = 3600;
                } else if (day > transition_sunday) {
                    if (3 == month) delta = 3600;
                } else { // day = transition_sunday
                    if (3 == month){
                        if (hour > 2)delta = 3600;
                    } else { // month = 10
                        if (hour < 2) delta = 3600;
                    }
                } // end transition_sunday
            } // end march or november
        } else {
            delta = 0;
        }
    }
    return delta;
}

int32_t rtc_tz_delta(uint32_t secs){
    uint32_t tz_delta = get_tz_delta_def(timezone_i);
    uint32_t time_no_dst = secs + tz_delta;
    return tz_delta + get_tz_delta_dst(time_no_dst, timezone_i);
}

void rtc_set_tz(timezone_t timezone){
    timezone_i = timezone;
}

timezone_t rtc_get_tz(void){
    return timezone_i;
}

uint32_t rtc_time_secs_get_tz(void){
    uint32_t secs = rtc_time_secs_get();
    return secs + rtc_tz_delta(secs); 
}

int days_in_month(int month,int year){
    int i=31;
    if ((month==4)||(month==6)||(month==9)||(month==11)) i=30;
    if (month==2){ if (year&0x03) i=28; else i=29;}
    return i;
}

void rtc_secs_to_hms(uint32_t secs, uint8_t * phour, uint8_t * pmin, uint8_t * psec){
    uint32_t t = secs % SECS_IN_DAY;
    if (psec) *psec = t % 60;
    t /= 60;
    if (pmin) *pmin = t % 60;
    if (phour) *phour = t / 60;
}

void rtc_days_to_date(uint32_t days, uint8_t * pday, uint8_t * pmonth, uint8_t * pyear){
    int y4 = days / (365 * 4 + 1);
    int r4 = days % (365 * 4 + 1);
    int year = 4 * y4;
    days -= y4 * (365 * 4 + 1);
    if (r4 > 365) {
        int dy = ((r4 - 1) / 365); 
        year += dy;
        days -= (1 + 365 * dy);
    }
    int day = 1 + days;
    int month = 1;
    while (month < 12){
        int dy = days_in_month(month, year);
        if (day > dy){
            ++month;
            day -= dy;
        }else{
            break;
        }
    }
    if (pday) *pday = day;
    if (pmonth) *pmonth = month;
    if (pyear) *pyear = year;
}

uint32_t rtc_hms_to_secs(int hour, int min, int sec){
    return sec + 60 * (min + 60 * hour);
}

uint32_t rtc_date_to_days(int day, int month, int year){
    uint32_t days = 365 * year;
    days += (3 + year) / 4;
    for (int m = 1; m < month; ++m) days += days_in_month(m, year);
    days += day - 1;
    return days;
}

void rtc_get_hms(uint8_t * phour, uint8_t * pmin, uint8_t * psec){
    return rtc_secs_to_hms( rtc_time_secs_get_tz(), phour, pmin, psec);
}

void rtc_get_date(uint8_t * pday, uint8_t * pmonth, uint8_t * pyear){
    uint32_t days = (rtc_time_secs_get_tz()) / SECS_IN_DAY;
    return rtc_days_to_date(days, pday, pmonth, pyear);
}

void rtc_get_date_time_from_secs(uint32_t secs, uint8_t * pday, uint8_t * pmonth, uint8_t * pyear, uint8_t * phour, uint8_t * pmin, uint8_t * psec){
    rtc_days_to_date(secs / SECS_IN_DAY, pday, pmonth, pyear);
    return rtc_secs_to_hms(secs, phour, pmin, psec);
}


uint8_t rtc_get_dow_from_secs(uint32_t secs){
    uint32_t day = secs / (SECS_IN_DAY);
    uint8_t dow = 1 + (5 + day) % 7; // 1st Jan 2000 was Saturday
    return dow;
}

void _rtc_set_date_time(int day, int month, int year, int hour, int min, int sec, int tz_flag){
    uint32_t secs = rtc_hms_to_secs(hour, min, sec) + SECS_IN_DAY * rtc_date_to_days(day, month, year);
    if (tz_flag) secs -= rtc_tz_delta(secs);
    ticks_at_ref = rtc_platform_ticks_get();
    secs_from_start_to_ref = secs;
}

void _rtc_get_date_time_dow(uint8_t * pday, uint8_t * pmonth, uint8_t * pyear, uint8_t * phour, uint8_t * pmin, uint8_t * psec, uint8_t * pdow, int tz_flag){
    uint32_t secs = (tz_flag) ? rtc_time_secs_get_tz() : rtc_time_secs_get();
    if (pdow) *pdow = rtc_get_dow_from_secs(secs);
    return rtc_get_date_time_from_secs(secs, pday, pmonth, pyear, phour, pmin, psec);    
}

const tzNameVal_t * const tzVariants[] __attribute((section(".text"))) = {
    &(tzNameVal_t){" US Pacific",     {-32, dst_mode_US}},
    &(tzNameVal_t){" US Mountain",    {-28, dst_mode_US}},
    &(tzNameVal_t){" US Central",     {-24, dst_mode_US}},
    &(tzNameVal_t){" US Eastern",     {-20, dst_mode_US}},
    &(tzNameVal_t){" UTC",            {0,   dst_mode_none}},
    &(tzNameVal_t){" EU/Berlin",      {4,   dst_mode_EU}},
    &(tzNameVal_t){" Kaliningrad",    {8,   dst_mode_none}},
    &(tzNameVal_t){" Kiev",           {8,   dst_mode_SU}},
    &(tzNameVal_t){" Moscow",         {12,  dst_mode_none}},
    &(tzNameVal_t){" Samara",         {16,  dst_mode_none}},
    &(tzNameVal_t){" India",          {18,  dst_mode_none}},
    &(tzNameVal_t){" Ekaterinburg",   {20,  dst_mode_none}},
    &(tzNameVal_t){" Omsk",           {24,  dst_mode_none}},
    &(tzNameVal_t){" Krasnoyarsk",    {28,  dst_mode_none}},
    &(tzNameVal_t){" Irkutsk",        {32,  dst_mode_none}},
    &(tzNameVal_t){" Singapore",      {32,  dst_mode_none}},
    &(tzNameVal_t){" Yakutsk",        {36,  dst_mode_none}},
    &(tzNameVal_t){" Vladivostok",    {40,  dst_mode_none}},
    &(tzNameVal_t){" Magadan",        {44,  dst_mode_none}},
    &(tzNameVal_t){" Kamchatka",      {48,  dst_mode_none}},
    NULL,
};

int getTzIndexByTz(timezone_t tz){
    int i = 0;
    do {
        if ((tz.delta_QH == tzVariants[i]->tz.delta_QH) && (tz.dst_mode == tzVariants[i]->tz.dst_mode)) return i;
    } while (tzVariants[++i]);
    return -1;
}