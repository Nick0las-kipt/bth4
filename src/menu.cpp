#include <zephyr.h>
#include <stdio.h>
#define LOG_LEVEL 4
#include <logging/log.h>
LOG_MODULE_REGISTER(menu);
#include <logging/log_ctrl.h>
#include "menu.h"
#include "ui-config.h"
#include "rtc.h"
#include "bt.h"
#include "backlight.h"
#include "dispservice.h"
#include "calculs/meteomath.h"
#include "slowsensors.h"
#include "calculs/daynightimpl.h"

#if UI_PRESENT
// Forward declarations
static void nLoopReset();
static void setR(uint16_t R);
static void setTimePr();
static void setAlarmPr();
static void setDatePr();
static void setDateAlarm();
static void setRmin(uint16_t val);
static void setRmax(uint16_t val);
static void incdate();
static bool listnotify(uint8_t num, uint8_t ncase);
static void setClock();
static void unpairBt();
static int setBtAdv(int * status);
static int setTzByIndex(int * index);
static int setBlMode(int * index);
static int setBlTimeoutSecs(int * secs);
static void ksSetLat(int16_t newval);
static void ksSetLong(int16_t newval);

static int32_t blLimLux = 100;

static MSMintparam<uint8_t,3,5> testparam0(1,22,4);

#define SYMS_TZ   14
#define SYMS_BT   6
#define SYMS_BL_MODE 7
typedef MSMintitemCG<int> MSMItemIntCG;
typedef MSMnameditemCG<int> MSMItemNameCG;

typedef MSMintitemC<int16_t,5,4>    MSMItemLLParam;
typedef MSMintitemP<int32_t, 5,0>   MSMItemI5ParamP;

typedef struct hist_pres_s {
    uint32_t pressure;
    uint32_t timestamp;
} hist_pres_t;

#define HIST_TAB_COUNT 6

static hist_pres_t histTab[HIST_TAB_COUNT];

static void histTabInsert(uint32_t pressure);

#define PR_W 6
#define ALT_W 4

class MSMhistpres:public MSMItem{
  public:
  MSMhistpres(const char* name, uint8_t newflags, hist_pres_t * element) : MSMItem(newflags,name,element){}

  virtual void getName(char* str,uint8_t lLim){
    loadName(str,lLim);
    strFix(str,lLim);
    hist_pres_t * pelement = (hist_pres_t*) _rptr;
    if ((pelement) && (pelement->pressure)){
        if (lLim >= PR_W) itoSPadded(pelement->pressure, &str[lLim - PR_W], PR_W, 4);
        if (lLim >= PR_W+11){
            uint32_t corrSecs = pelement->timestamp + rtc_tz_delta(pelement->timestamp);
            uint8_t day, month, hour, minute;
            rtc_get_date_time_from_secs(corrSecs, &day, &month, NULL, &hour, &minute, NULL);
            itoSPadded(hour, &str[lLim - PR_W - 12], 2, 0xff);
            str[lLim - PR_W - 10] = MSM_TIME_SEPARATOR;
            itoSPadded(minute, &str[lLim - PR_W - 9], 2, 0xff);
            str[lLim - PR_W - 7] = ' ';
            itoSPadded(day, &str[lLim - PR_W - 6], 2, 0xff);
            str[lLim - PR_W - 4] = MSM_DATE_SEPARATOR;
            itoSPadded(month, &str[lLim - PR_W - 3], 2, 0xff);
            str[lLim - PR_W - 1] = ' ';
        }
    } else {
        memset(str, '-', lLim);
    }
  }

  virtual void Enter(){
  }

  virtual bool passKeys(uint8_t key,uint8_t count){
    return false;
  }

  virtual void getSelection(uint8_t &firstp, uint8_t &lastp,uint8_t lLim){
    firstp = 0;
    lastp = lLim - 1;
  }
};

class MSMgrpitem:public MSMItem{
  public:
  MSMgrpitem(const char* name, uint8_t newflags, void(*func)(uint32_t)) : MSMItem(newflags,name,(void*)func),_actVal(0){}

  virtual void callUpdate(){
    setGlobalReferencePressure(_actVal);
    void(*func)(uint32_t) = (void(*)(uint32_t))_rptr;
    if (func) func(_actVal);
  }
  virtual void getName(char* str,uint8_t lLim){
    loadName(str,lLim);
    strFix(str,lLim);
    uint32_t actVal = _actVal ? _actVal: getGlobalReferencePressure();
    if (lLim >= PR_W) itoSPadded(actVal,&str[lLim - PR_W], PR_W, 4);
    if (lLim > PR_W + 2 + ALT_W){
        uint32_t p = slowsensorsGetPressure(FLAG_SENSOR_PRESSURE_SMOOTH | FLAG_SENSOR_PRESSURE_MMHG);
        uint32_t t = slowsensorsGetTemperature(FLAG_SENSOR_TEMPERATURE_BAROSENSOR);

        uint32_t alt = getBaroAlt(p, actVal, t);
        itoSPadded(alt, &str[lLim - PR_W - 2 - ALT_W], ALT_W, 0);
        str[lLim -PR_W - 2] = 'm';
        str[lLim -PR_W - 1] = '/';
    }
  }

  virtual void Enter(){
    _actVal = getGlobalReferencePressure();
    setUpdated();
  }

  virtual bool passKeys(uint8_t key,uint8_t count){
    setUpdated();
    if((key == MSM_KEY_ENTER) && (count == 0)){
      callUpdate();
      _actVal = 0;
      return false;
    }

    if(key == MSM_KEY_ESC){
      _actVal = 0;
      return false;
    }
    uint32_t delta;

    convertCount(count,delta);

    if(key == MSM_KEY_UP){
      if(( PREF_MAX_LIM-delta) > _actVal) _actVal += delta; else _actVal = PREF_MAX_LIM;
    }
    if(key == MSM_KEY_DOWN){
      if(_actVal > PREF_MIN_LIM+delta) _actVal -= delta; else _actVal = PREF_MIN_LIM;
    }
    return true;
  }

  virtual void getSelection(uint8_t &firstp, uint8_t &lastp,uint8_t lLim){
    firstp = lLim - PR_W;
    lastp = lLim - 1;
  }

  uint32_t getVal(){
    return _actVal;
  }

protected:
  uint32_t _actVal;
};

class MSMdnDisplay: public MSMItem{
public:
    MSMdnDisplay(const char * name, float * dataSource, bool adjustTz=false):MSMItem(name,MSM_MODE_NORMAL),dataSource(dataSource){
        if (adjustTz) {
            _rptr = (void*) 0x400;
        } else {
            _rptr = NULL;
        }
    }
    virtual void getName(char* str,uint8_t lLim){
    loadName(str, lLim);
    strFix(str, lLim);
    if (lLim >=5){
        float src = *dataSource;
        int8_t data = src;
        if ((0 <= data) && (data <24)){
            if (_rptr){
                uint32_t atime =rtc_time_secs_get();
                atime -= atime % 86400;
                atime += 3600.0f*src;
                int32_t tz_delta_secs = rtc_tz_delta(atime);
                float deltaTz = (float)tz_delta_secs/3600.0f;
                src = fmod24(src+deltaTz);
                data = src;
            }
            itoSPadded(data,&str[lLim-5],2,5);
            data = (src - (float)data)*60.0f;
            itoSPadded(data,&str[lLim-2],2,5);
        }else{
            char chFill = (data > 0) ? '^' : '_';
            str[lLim-5]=chFill;str[lLim-4]=chFill;
            str[lLim-2]=chFill;str[lLim-1]=chFill;
        }
        str[lLim-3]=MSM_TIME_SEPARATOR;
    }
  }
private:
    float * dataSource;
};


// Names

MSMStr mainMenuName      __attribute((section(".text"))) = "The main menu";
MSMStr clockSubMenuName  __attribute((section(".text"))) = "Clock";
MSMStr btSubMenuName     __attribute((section(".text"))) = "Bluetooth";
MSMStr altSubMenuName    __attribute((section(".text"))) = "Altimeter";
MSMStr demoSubMenuName   __attribute((section(".text"))) = "Demo menu";

MSMStr clockItemName     __attribute((section(".text"))) = "Set clock";
MSMStr dateItemName      __attribute((section(".text"))) = "Set date";
MSMStr setClockName      __attribute((section(".text"))) = "Set clock";
MSMStr setTzName         __attribute((section(".text"))) = "TZ: ";
MSMStr btUnpairName      __attribute((section(".text"))) = "Unpair all";
MSMStr btAdvName         __attribute((section(".text"))) = "Advertisment";

MSMStr altSetRefPresName __attribute((section(".text"))) = "Ref:";
MSMStr histRefPresName   __attribute((section(".text"))) = " ";

MSMStr mItemDayNightName __attribute((section(".text"))) = "Day/Night";
MSMStr mItemLatName      __attribute((section(".text"))) = "Lat:";
MSMStr mItemLongName     __attribute((section(".text"))) = "Long:";
MSMStr mItemTzName       __attribute((section(".text"))) = "Time Zone:";
MSMStr mItemSunriseName  __attribute((section(".text"))) = "Rise:";
MSMStr mItemSunsetName   __attribute((section(".text"))) = "Set:";
MSMStr mItemDuskCivil    __attribute((section(".text"))) = "Civil dt:";
MSMStr mItemDuskNautical __attribute((section(".text"))) = "Nautical dt:";
MSMStr mItemDuskAstro    __attribute((section(".text"))) = "Astro dt:";

MSMStr mItemBlName       __attribute((section(".text"))) = "Backlight";
MSMStr mItemBlModeName   __attribute((section(".text"))) = "Mode:";
MSMStr mItemBlLuxLimName __attribute((section(".text"))) = "LUX limit:";
MSMStr mItemBlToName     __attribute((section(".text"))) = "Timeout:";

MSMStr minRname          __attribute((section(".text"))) = "Minimum R";
MSMStr maxRname          __attribute((section(".text"))) = "Maximum R";
MSMStr setRname          __attribute((section(".text"))) = "Set R";
MSMStr subMenuName       __attribute((section(".text"))) = "The Submenu";
MSMStr dtaName           __attribute((section(".text"))) = "Days to add";
MSMStr alarmItemName     __attribute((section(".text"))) = "Alarm on";
MSMStr alarmDItemName    __attribute((section(".text"))) = "Date: ";
MSMStr addDaysName       __attribute((section(".text"))) = "Add days";
MSMStr someSub0Name      __attribute((section(".text"))) = "Useless line";
MSMStr someSub1Name      __attribute((section(".text"))) = "Submenu back";
MSMStr someSub2Name      __attribute((section(".text"))) = "Submenu close all";
MSMStr someSubXName      __attribute((section(".text"))) = "The same submenu";

/*MSMStr mSelectionOn     __attribute((section(".text"))) = " On";
MSMStr mSelectionOff    __attribute((section(".text"))) = " Off";

static const char * const offOnNameArray[]={
  (const char* const) &mSelectionOff,
  (const char* const) &mSelectionOn,
  (const char* const) 0,
};*/

static const char * const offOnNameArray[] = {
    " Off",
    " On",
    NULL,
};

static const char * const blModeNameArray[] = {
    " Off",
    " Dark",
    " Night",
    " On",
    NULL,
};

enum blMode_e{
    blModeOff = 0,
    blModeDark = 1,
    blModeNight = 2,
    blModeOn = 3,
};

char loopBuf[32];
// Menu Items
MSMItem mainMenuTitle(&mainMenuName[0],0);
MSMItem clockSubMenuItem((char*)&clockSubMenuName[0],MSM_MODE_SUBMENU);
MSMItem btSubMenuItem((char*)&btSubMenuName[0],MSM_MODE_SUBMENU);
MSMItem altSubMenuItem((char*)&altSubMenuName[0],MSM_MODE_SUBMENU);
MSMItem demoSubMenuItem((char*)&demoSubMenuName[0],MSM_MODE_SUBMENU);

MSMtimeitemC clockItem((char*)&clockItemName[0],MSM_MODE_OP,12,34,56,setTimePr);
MSMdateitemC dateItem((char*)&dateItemName[0],MSM_MODE_OP,27,12,15,setDatePr);
MSMItem setClockItem((char*)&setClockName[0],MSM_MODE_CLOSEALL,setClock);
MSMItemNameCG setTzItem((char*)&setTzName[0],MSM_MODE_OP, SYMS_TZ, setTzByIndex, (const char * const * )&tzVariants[0]);

MSMItem btUnpairItem((char*)&btUnpairName[0],MSM_MODE_CLOSEALL, unpairBt);
MSMItemNameCG btAdvItem((char*)&btAdvName[0],MSM_MODE_OP, SYMS_BT, setBtAdv, &offOnNameArray[0]);

MSMgrpitem altSetRefPresItem((char*)&altSetRefPresName[0], MSM_MODE_OP, histTabInsert);
MSMhistpres histItem0((char*)&histRefPresName[0], 0, &histTab[0]);
MSMhistpres histItem1((char*)&histRefPresName[0], 0, &histTab[1]);
MSMhistpres histItem2((char*)&histRefPresName[0], 0, &histTab[2]);
MSMhistpres histItem3((char*)&histRefPresName[0], 0, &histTab[3]);
MSMhistpres histItem4((char*)&histRefPresName[0], 0, &histTab[4]);
MSMhistpres histItem5((char*)&histRefPresName[0], 0, &histTab[5]);

//Day/Night Items
MSMItem          mItemDayNight(&mItemDayNightName[0],MSM_MODE_SUBMENU);
MSMItemLLParam   mItemLat(&mItemLatName[0],MSM_MODE_NORMAL,-899,899,0,ksSetLat);
MSMItemLLParam   mItemLong(&mItemLongName[0],MSM_MODE_NORMAL,-1800,1800,0,ksSetLong);
MSMdnDisplay     mItemShowSunrise(&mItemSunriseName[0],&dn_gap[DAY_NIGHT_RISE_N].start,true);
MSMdnDisplay     mItemShowSunset(&mItemSunsetName[0],&dn_gap[DAY_NIGHT_RISE_N].end,true);

MSMdnDisplay     mItemShowDuskCivil(&mItemDuskCivil[0],&civilLength);
MSMdnDisplay     mItemShowDuskNautical(&mItemDuskNautical[0],&nauticalLength);
MSMdnDisplay     mItemShowDuskAstro(&mItemDuskAstro[0],&astroLength);

MSMItem          mItemBl(&mItemBlName[0], MSM_MODE_SUBMENU);
MSMItemNameCG    mItemBlMode((char*)&mItemBlModeName[0], MSM_MODE_OP, SYMS_BL_MODE, setBlMode, &blModeNameArray[0]);
MSMItemI5ParamP  mItemBlLuxLim((char*)&mItemBlLuxLimName[0], MSM_MODE_OP, 1, 99999, &blLimLux);
MSMItemIntCG     mItemBlTimeout((char*)&mItemBlToName, MSM_MODE_OP, 2, 60, setBlTimeoutSecs, 5, 0);
             
MSMintitemC<uint16_t,5,3> setRitem(&setRname[0],0,1,2000,10,setR);    //set R
MSMintitemC<uint16_t,5,3> minRitem(&minRname[0],0,1,2000,setRitem.getMin(),setRmin);    //set min R 
MSMintitemC<uint16_t,5,3> maxRitem(&maxRname[0],0,1,2000,setRitem.getMax(),setRmax);    //set max R

MSMtimeitemC alarmItem((char*)&alarmItemName[0],MSM_MODE_OP,12,35,06,setAlarmPr);
MSMdateitemC alarmdItem((char*)&alarmDItemName[0],MSM_MODE_OP,27,12,18,setDateAlarm);

MSMItem subMenuItem((char*)&subMenuName[0],MSM_MODE_SUBMENU);
MSMItem dtaItem((char*)&dtaName[0],MSM_MODE_OP,&testparam0);
MSMItem addDaysItem((char*)&addDaysName[0],MSM_MODE_NORMAL,incdate);
MSMItem cycleNumItem((char*)&loopBuf[0],MSM_FL_LIVEUPDATE|MSM_FL_RAMNAME,nLoopReset);
MSMItem someSub0Item((char*)&someSub0Name[0],0);
MSMItem someSub1Item((char*)&someSub1Name[0],MSM_MODE_CLOSELEVEL);
MSMItem someSub2Item((char*)&someSub2Name[0],MSM_MODE_CLOSEALL,nLoopReset);
MSMItem someSubXItem((char*)&someSubXName[0],MSM_MODE_SUBMENU);

MSMList mainMenuList(listnotify,&mainMenuTitle,&mItemBl,&altSubMenuItem,&mItemDayNight,&clockSubMenuItem,&btSubMenuItem,&demoSubMenuItem);
MSMList clockMenuList(0, &clockSubMenuItem,&clockItem,&dateItem,&setClockItem,&setTzItem);
MSMList blMenuList(0, &mItemBl,&mItemBlMode, &mItemBlLuxLim, &mItemBlTimeout);
MSMList btMenuList(0, &btSubMenuItem, &btAdvItem, &btUnpairItem);
MSMList altMenuList(0, &altSubMenuItem, &altSetRefPresItem, &histItem0, &histItem1, &histItem2, &histItem3, &histItem4, &histItem5);
MSMList dayNightMenuList(0,
        &mItemDayNight,
        &mItemLat,
        &mItemLong,
        &mItemShowSunrise,
        &mItemShowSunset,
        &mItemShowDuskCivil,
        &mItemShowDuskNautical,
        &mItemShowDuskAstro);

MSMList demoMenuList(0,&demoSubMenuItem,&minRitem,&maxRitem,&setRitem,&subMenuItem,&alarmItem,&alarmdItem,&dtaItem,&addDaysItem);
MSMList someSubMenuList(0,&subMenuItem,&someSub0Item,&someSub1Item,&someSub2Item);

MSMDisplay menuDisplay(MENU_LINES);

static void histTabInsert(uint32_t pressure){
    if (pressure == histTab[0].pressure) return;
    for(int i = sizeof(histTab)/sizeof(histTab[0]); i > 0 ; --i){
        histTab[i] = histTab [i-1];
    }
    histTab[0].timestamp = rtc_time_secs_get();
    histTab[0].pressure = pressure;
    altMenuList.setUpdatedFrom(0);
}

static int blMode = 0;
static int setBlMode(int * index){
    if (index) blMode = *index;
    return blMode;
}

static int setBlTimeoutSecs(int * secs){
    if (secs) lcd_bl_set_timeout_ms(*secs * 1000U);
    return (500 + lcd_bl_get_timeout_ms()) / 1000U;
}

bool shouldProlongateBl(){
    if (blModeOn == blMode) {
        return true;
    } else if (blModeOff == blMode) {
        return false;
    } else if (blModeDark == blMode){
        return (slowsensorsGetLux(0,0)/10000 < (uint32_t)blLimLux);
    } else if (blModeNight == blMode) {
        uint8_t hour, min;
        rtc_secs_to_hms(rtc_time_secs_get(), &hour, &min, NULL);
        return !isDay(hour, min);
    }
    return false;
}

int loopN = 0;

static void nLoopReset(){
    loopN=0;
}

static void setR(uint16_t R){
     LOG_DBG("R is set to %d",R);
}

static void setTimePr(){
    char buf[32];
    clockItem.timeToStr(buf);
    LOG_DBG("Time is set to %s",log_strdup(buf));
}

static void setAlarmPr(){
    char buf[32];
    alarmItem.timeToStr(buf);
    LOG_DBG("Time is set to %s",log_strdup(buf));
}

static void setDatePr(){
    LOG_DBG("Date set to %d/%d/%d",dateItem.getDay(),dateItem.getMonth(),dateItem.getYear());
}

static void setDateAlarm(){
    LOG_DBG("Alarm date set to %d/%d/%d",dateItem.getDay(),dateItem.getMonth(),dateItem.getYear());
}

static void clockPreload(){
    uint8_t hour, min, sec, day, month, year;
    rtc_get_date_time_from_secs(20 + rtc_time_secs_get_tz(), &day, &month, &year, &hour, &min, &sec);
    clockItem.setTime(hour, min, sec);
    dateItem.setDate(day, month, year);
}

static void confPreload(){
    btAdvItem.setVal(bt_adv_status_get());
}

static void setRmin(uint16_t val){setRitem.setMin(val);}
static void setRmax(uint16_t val){setRitem.setMax(val);}
static void incdate(){alarmdItem.incDate(testparam0.getVal());}

static void setClock(){
    uint8_t hour, min, sec, day, month, year;
    clockItem.getTime(hour, min, sec);
    dateItem.getDate(day, month, year);
    rtc_set_date_time_tz(day, month, year, hour, min, sec);
}

static int getTzIndexForMenu(){
    int index = getTzIndexByTz(rtc_get_tz());
    if (index < 0) index = getTzIndexByTz({0, 0});
    if (index < 0) index = 0;
    return index;
}

static int setTzByIndex(int * index){
    if (index) {
      rtc_set_tz(tzVariants[*index]->tz);
      return *index;
    } else {
        timezone_t ctz = rtc_get_tz();
        for(int i = 0;(tzVariants[i]); ++i){
            if (0 == memcmp(&ctz, &tzVariants[i]->tz, sizeof(timezone_t))){
                return i;
            }
        }
    }
    return 0;
}

static void unpairBt(){
    int rv = bt_unpair_all();
    LOG_DBG("BT Unpair result: %d", rv); 
}

static int setBtAdv(int * status){
    if (status){
        int rv = bt_adv_enable_set(*status);
        LOG_DBG("BT ADV enable: status=%d result=%d", *status, rv);
    }
    return bt_adv_status_get();
}

void updateLoopNum(){
        ++loopN;
        snprintf(loopBuf,sizeof(loopBuf), "Loop num %d",loopN);
}

static void setDayNightUpdated(){
    mItemShowSunrise.setUpdated();
    mItemShowSunset.setUpdated();
    mItemShowDuskCivil.setUpdated();
    mItemShowDuskNautical.setUpdated();
    mItemShowDuskAstro.setUpdated();
}
static void ksSetLat(int16_t newval){
    dn_latitude = newval/10.0f;
    updateDayNightForce();
    setDayNightUpdated();
}

static void ksSetLong(int16_t newval){
    dn_longitude = newval/10.0f;
    updateDayNightForce();
    setDayNightUpdated();
}

static bool listnotify(uint8_t num,uint8_t ncase){
    updateDayNightForce();
    setDayNightUpdated();
    LOG_DBG("List Notify %d, %d",num, ncase);
    return true;
}

void menuLateInitDemo(){
        someSubXItem.setList(&someSubMenuList);
        someSubMenuList.addItem(&someSubXItem,1);
        someSubMenuList.addItem(&someSub0Item,2);
        someSubMenuList.addItem(&someSub0Item,4);
        mainMenuList.addItem(&cycleNumItem,3); 
}

void menuOpen(){
    clockPreload();
    confPreload();
    setTzItem.setVal(getTzIndexForMenu());
    menuDisplay.reset(&mainMenuList);
}

bool menuProcessKeys(uint8_t key, uint8_t count){
    menuDisplay.ProcessKeys(key,count);
    return menuDisplay.getUpdatedAny();
}

bool menuIsOpen(){
    return menuDisplay.isopen();
}

MSMDisplay * getMenuDisplay(){
    return &menuDisplay;
}

#endif