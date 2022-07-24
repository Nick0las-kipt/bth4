#include <zephyr.h>
#include <sys/printk.h>
#include <kernel.h>

#define LOG_LEVEL 4
#include <logging/log.h>
LOG_MODULE_REGISTER(app);

#include <drivers/adc.h>
#include <drivers/gpio.h>
#include <drivers/pwm.h>
#include "lpm013m126.h"
#include "backlight.h"
#include "slowsensors.h"
#include "fonts.h"
#include "buttons.h"
#include "bt.h"
#include "menu.h"
#include "rtc.h"
#include "ui-config.h"
#include "dispservice.h"
#include "watchdog.h"
#include "graph.h"
#include "sensorlog.h"

#define CONSOLE_LABEL DT_LABEL(DT_CHOSEN(zephyr_console))

#if UI_PRESENT
typedef enum {
    dm_digital_clock = 0,
    dm_meteo_graph_fast = 1,
    dm_meteo_graph_hist = 2,
} display_mode_e;

typedef bool (*drawer_f)(bool);
typedef bool (*keyProcessor_f)(int, int);
typedef void (*activator_f)(int);

typedef struct displayHandler_s {
    drawer_f drawer;
    keyProcessor_f keyProcessor;
    activator_f activator;
} displayHandler_t;

static const displayHandler_t displayDispatch[] = {
    {
        .drawer = displayClockTimeAndPressure,
    },
    {
        .drawer = drawGraphFast,
        .keyProcessor = graphFastKeys,
        .activator = slowsensorsSetFastRun,
    },
    {
        .drawer = drawGraphHist,
        .keyProcessor = graphHistKeys,
    },
    {
        .drawer = displayLuxInfo,
        .activator = slowsensorsSetFastRun,
    },    
};

static const uint8_t keyMap[] = {MSM_KEY_ESC,MSM_KEY_ENTER,MSM_KEY_UP,MSM_KEY_DOWN};

#define COUNTS_TO_DROP 2

void uiDropKeyCount(int32_t *key, int32_t *count){
    if (*count) {
        if (*count > COUNTS_TO_DROP){
            *count -= COUNTS_TO_DROP;
        }else {
            *key = 0;
        }
    }
}

static bool uiDisplayIdle(bool force, unsigned int dm){
#ifdef CONFIG_LOG    
    uint32_t before = k_cycle_get_32();
#endif
    if (dm >= sizeof(displayDispatch)/sizeof(displayDispatch[0])){
        LOG_ERR("Wrong display mode %d, set to 0", dm);
        dm = 0;
    }
    bool res = displayDispatch[dm].drawer ? displayDispatch[dm].drawer(force) : false;
#ifdef CONFIG_LOG    
    if (res){
        uint32_t after = k_cycle_get_32();
        LOG_DBG("Idle screen %d drawn in %dms", dm, (1000U*(after - before))/sys_clock_hw_cycles_per_sec());
    }
#endif
    return res;
}

static void uiActivate(int arg, unsigned int dm){
    if (dm >= sizeof(displayDispatch)/sizeof(displayDispatch[0])){
        LOG_ERR("Wrong display mode %d, ignore", dm);
        return;
    }
    if (displayDispatch[dm].activator) displayDispatch[dm].activator(arg);
}

static bool uiKeyProcess(int key, int count, unsigned int dm){
    if (dm >= sizeof(displayDispatch)/sizeof(displayDispatch[0])){
        LOG_ERR("Wrong display mode %d, ignore", dm);
        return false;
    }
    return displayDispatch[dm].keyProcessor ? displayDispatch[dm].keyProcessor(key, count) : false;
}

static void uiLoopRun()
{
    int32_t keyin,key,count;
    unsigned int displayModeIndex = 0;
    MSMDrawer drawer(DEF_MENU_PAD + CHAR_W, MIN_LINE_DO + DEF_MENU_PAD, MENU_CHARS_IN_LINE, COL_MENU_TEXT_FRONT, COL_MENU_TEXT_BACK, COL_MENU_SEL_FRONT, COL_MENU_SEL_BACK, COL_MENU_BKG);
    menuLateInitDemo();
    uiActivate(1, displayModeIndex);
    while (1){
        process_button_status(&keyin, &count, 1);
        bool update = false;
        if (keyin >= 0){
            key = keyMap[keyin];
            count--; // compatibility buttons to MSM;
            uiDropKeyCount(&key, &count);
            if (shouldProlongateBl()) lcd_bl_prolongate(1);
            //update = true;
        }else{
            key = 0;
            count = 0;
        }
        if (!(menuIsOpen())){
            if ((MSM_KEY_ENTER == key) && (0 == count)) menuOpen();
            bool keyProcessF = false;
            if ((MSM_KEY_DOWN == key) || (MSM_KEY_UP == key) || (MSM_KEY_ESC == key)) {
                keyProcessF =  uiKeyProcess(key, count, displayModeIndex);
            }            
            if (!keyProcessF && (MSM_KEY_ESC == key) && (0 == count)) {
                uiActivate(0, displayModeIndex);
                if (++displayModeIndex >= sizeof(displayDispatch)/sizeof(displayDispatch[0])) displayModeIndex = 0;
                update = true;
                uiActivate(1, displayModeIndex);
            }

        }else{
           update = menuProcessKeys(key,count);
        }
        if (key) LOG_DBG("Key %d count %d done", key, count);
        if (menuIsOpen()){
            uint32_t before = k_cycle_get_32();
            drawer.draw(getMenuDisplay());
            uint32_t after = k_cycle_get_32();
            int ms = (1000U*(after - before))/sys_clock_hw_cycles_per_sec();
            if (ms) LOG_DBG("Menu drawn in %dms", ms);
        } else {
            uiDisplayIdle(update, displayModeIndex);
        }
        
        updateLoopNum(); // TODO: remove after demo removed
        update |= displayStatusBar(false);
        update |= displayInfoBar(false);

        wdt_prolongate();
        int delay_ms = key ? 100 : 500;        
        k_sleep(K_MSEC(delay_ms));
    }
}
#endif

void main(void)
{
    int ret;
    wdt_configure();
    lcd_disoff();
    buttons_init();
    rtc_init();
    rtc_set_date_time(20,07,22,21,00,00);
    sensorLogFill();
    bt_init();
    for(int i = 0; i < 10; ++i){
        k_sleep(K_MSEC(100));
        wdt_prolongate();
    }
    slowsensors_init();

    ret = lcd_init();
    lcd_bl_init();
    lcd_clear();
    buttons_set_listener(k_current_get());
    for(int i = 0; i < 10; ++i){
        k_sleep(K_MSEC(100));
        wdt_prolongate();
    }
#if (not defined(CONFIG_LOG)) && (not defined(CONFIG_SHELL))
    struct device *cons = device_get_binding(CONSOLE_LABEL);
    device_set_power_state(cons, DEVICE_PM_LOW_POWER_STATE, NULL, NULL);
#endif
#if UI_PRESENT
    uiLoopRun();
#endif
    while(1){
        k_sleep(K_MSEC(100));
        wdt_prolongate();        
    }
}
