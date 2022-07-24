#include <zephyr.h>
#include <kernel.h>
#include <drivers/adc.h>
#include <drivers/gpio.h>
#include <init.h>


#define CHG_NODE DT_NODELABEL(charger_gpio)
#define CHG_NAME DT_LABEL(CHG_NODE)

#if DT_NODE_EXISTS(CHG_NODE)

static struct device * pg_dev;
static struct device * chg_dev;

static int charger_gpio_init(struct device * dptr){
    (void) dptr;
#if DT_NODE_EXISTS(DT_PROP(CHG_NODE, charge_gpios))
    chg_dev = device_get_binding(DT_GPIO_LABEL(CHG_NODE, charge_gpios));
    if (chg_dev){
       gpio_pin_configure(chg_dev, DT_GPIO_PIN(CHG_NODE, charge_gpios), 
             GPIO_INPUT | DT_GPIO_FLAGS(CHG_NODE, charge_gpios));
    }
#endif
#if DT_NODE_EXISTS(DT_PROP(CHG_NODE, power_good_gpios))
    pg_dev = device_get_binding(DT_GPIO_LABEL(CHG_NODE, power_good_gpios));
    if (pg_dev){
       gpio_pin_configure(pg_dev, DT_GPIO_PIN(CHG_NODE, power_good_gpios), 
             GPIO_INPUT | DT_GPIO_FLAGS(CHG_NODE, power_good_gpios));
    }
#endif    
    return 0;
}

int charger_is_charging(){
    if (chg_dev){
        int res = gpio_pin_get(chg_dev, DT_GPIO_PIN(CHG_NODE, charge_gpios));
        return res > 0;
    } else {
        return 0;
    }
}

int charger_power_good(){
    if (pg_dev){
        int res = gpio_pin_get(pg_dev, DT_GPIO_PIN(CHG_NODE, power_good_gpios));
        return res > 0;
    } else {
        return 0;
    }
}

// Power enable GPIO init should occur after GPIO but before drivers
SYS_INIT(charger_gpio_init, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE);

#else

int charger_is_charging(){
    return 0;
}

int charger_power_good(){
    return 0;
}

#endif