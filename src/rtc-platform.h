#ifndef RTC_PLATFORM_H
#define RTC_PLATFORM_H

// This works for NRF5x
#define PL_TICKS_HZ 1000
static inline uint64_t rtc_platform_ticks_get(void){return k_uptime_get();}
static inline int rtc_platform_init(void){ return 0;}
//TODO: support more platforms if needed

#endif
