#include <zephyr.h>
#include <kernel.h>
#include "rtc.h"
#include "ui-config.h"
#include "sensorlog.h"

#define LOG_LEVEL 2
#include <logging/log.h>
LOG_MODULE_REGISTER(sensorlog);
// graph data storage size
#ifndef GDS
  #define GDS (30*24*4) //2880 positions, 2min data for 4 days
#endif

#ifndef DATA_INTERVAL_SECS 
  #define DATA_INTERVAL_SECS (60 * LOG_MINUTE_INTERVAL)
#endif

static uint32_t grd[GDS]; // Graph data buffer
static uint32_t last_shift_secs;
static int last_log_pos;
static int log_num_used;

#define ST_RH_MASK    0xff
#define ST_TEMP_MASK 0x7ff
#define ST_TEMP_SHIFT    8
#define ST_P_MASK   0x1fff
#define ST_P_SHIFT      19

static void setLogRH(int pos,uint8_t aRH){
    grd[pos] = (grd[pos] & (~ST_RH_MASK)) + aRH;
}

uint8_t getLogRH(int pos){
    return grd[pos] & ST_RH_MASK;
}

static void setLogT(int pos,int16_t aT){
    uint32_t TT=((500+aT) & ST_TEMP_MASK) << ST_TEMP_SHIFT;     //shift temp
    grd[pos] = TT | (grd[pos] & (~(ST_TEMP_MASK << ST_TEMP_SHIFT)));
}

int16_t getLogT(int pos){
    int16_t i = (grd[pos] >> ST_TEMP_SHIFT) & ST_TEMP_MASK;
    return i-500;
}

static void setLogP(int pos,int16_t aP){
    uint32_t PP = ((aP) & ST_P_MASK) << ST_P_SHIFT;     //shift P
    grd[pos] = PP | (grd[pos] & (~(ST_P_MASK << ST_P_SHIFT)));
}

int16_t getLogP(int pos){
    return (grd[pos] >> ST_P_SHIFT) & ST_P_MASK;
}

uint32_t getLogPoint(int pos){
    return grd[pos];
}

int getLogSize(){
    return GDS;
}

int getLogUsed(){
    return log_num_used;
}

void getLogPosTime(int * posToStore, uint32_t *rtcTimeToStore){
    struct k_spinlock lock;  
    k_spinlock_key_t key;
    key = k_spin_lock(&lock);    
    if (posToStore) *posToStore = last_log_pos;
    if (rtcTimeToStore) *rtcTimeToStore = last_shift_secs;
    k_spin_unlock(&lock, key);
}

void sensorLogFill(){
    for (int i = 0; i < GDS; ++i){
        setLogRH(i, 10 + i % 80);
        setLogT(i, 200+i%100);
        setLogP(i, 7600-(i%50));
    }
    last_shift_secs = DATA_INTERVAL_SECS * (rtc_time_secs_get() / DATA_INTERVAL_SECS);
}

static int accuP;
static int accuT;
static int accuRH;
static int accuC;

int sensorLogProcessData(int pres, int temp, int rh){
    int rv = 0;
    accuP += pres;
    accuT += temp;
    accuRH += rh;
    accuC++;
    pres = (pres + 5) / 10;
    temp = (temp + ((temp >= 0) ? 5 : -5)) / 10;
    uint32_t p = getLogPoint(last_log_pos);
    setLogT(last_log_pos,temp); 
    setLogP(last_log_pos,pres);
    setLogRH(last_log_pos,rh);
    if (p != getLogPoint(last_log_pos)) rv = 1;
    uint32_t cur_secs = rtc_time_secs_get(); // LOG runs using UTC
    if ((cur_secs - last_shift_secs) >= DATA_INTERVAL_SECS){
        struct k_spinlock lock;  
        k_spinlock_key_t key;
        key = k_spin_lock(&lock);
        int avgT = accuT / accuC;
        int avgP = accuP / accuC;
        int avgRH = accuRH / accuC;
        avgP = (avgP + 5) / 10;
        avgT = (avgT + ((avgT >= 0) ? 5 : -5)) / 10;
        setLogT(last_log_pos,avgT); 
        setLogP(last_log_pos,avgP);
        setLogRH(last_log_pos,avgRH);
        accuP = 0;
        accuT = 0;
        accuRH = 0;
        accuC = 0;
        last_log_pos++;
        if(last_log_pos >= GDS) last_log_pos = 0;
        if (log_num_used < GDS) log_num_used++;
        setLogT(last_log_pos,temp); 
        setLogP(last_log_pos,pres);
        setLogRH(last_log_pos,rh);
        last_shift_secs = DATA_INTERVAL_SECS * (cur_secs / DATA_INTERVAL_SECS);
        rv = 3;
        k_spin_unlock(&lock, key);
    }
    return rv;
}
