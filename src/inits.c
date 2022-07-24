#include <zephyr.h>
#include <kernel.h>
#include <drivers/adc.h>
#include <drivers/gpio.h>
#include <init.h>

#define PWR_NODE DT_NODELABEL(poweron)
#define PWR_NAME DT_LABEL(PWR_NODE)

#if DT_NODE_EXISTS(PWR_NODE)

static int pwr_en_gpio_init(struct device * dptr){
     (void) dptr;
     struct device * pwr_en_gpio_dev = device_get_binding(DT_GPIO_LABEL(PWR_NODE, gpios));
     if (pwr_en_gpio_dev){
        const int pin = DT_GPIO_PIN(PWR_NODE, gpios);
        gpio_pin_configure(pwr_en_gpio_dev, pin, 
              GPIO_OUTPUT | DT_GPIO_FLAGS(PWR_NODE, gpios));
        gpio_pin_set(pwr_en_gpio_dev, pin, 1);
     }
     return 0;
}

// Power enable GPIO init should occur after GPIO but before drivers
SYS_INIT(pwr_en_gpio_init, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE);
#endif