#include <zephyr.h>
#include <device.h>
#include <drivers/watchdog.h>

#define LOG_LEVEL 2
#include <logging/log.h>
LOG_MODULE_REGISTER(wdt);

#if DT_NODE_HAS_STATUS(DT_ALIAS(watchdog0), okay)
#define WDT_NODE DT_ALIAS(watchdog0)
#define WDT_DEV_NAME DT_LABEL(WDT_NODE)
#endif

static struct device *wdt = NULL;
static int wdt_channel_id;

int wdt_configure(){
    int rv = -ENODEV;

#ifdef WDT_DEV_NAME
    do {
        wdt = device_get_binding(WDT_DEV_NAME);
        if (!wdt){
            LOG_ERR("Failed to get WDT binding for %s", WDT_DEV_NAME);
            break;
        }
        struct wdt_timeout_cfg wdt_config;
    
        wdt_config.flags = WDT_FLAG_RESET_SOC;
        wdt_config.window.min = 0U;
        wdt_config.window.max = 2000U;
    
        wdt_config.callback = NULL;

        wdt_channel_id = wdt_install_timeout(wdt, &wdt_config);

        if (wdt_channel_id < 0){
            LOG_ERR("Failed to install WDT timeout (%d)", wdt_channel_id);
            rv = wdt_channel_id;
            wdt = NULL;
            break;
        }
        rv = wdt_setup(wdt, WDT_OPT_PAUSE_HALTED_BY_DBG);
        if (rv < 0){
            LOG_ERR("Failed to setup WDT (%d)", rv);
            wdt = NULL;
            break;
        }
    } while (0);
#endif
    return rv;
}

int wdt_prolongate(){
    int rv = -ENODEV;
#ifdef WDT_DEV_NAME
        if (wdt) rv = wdt_feed(wdt, wdt_channel_id);
#endif
    return rv;
}