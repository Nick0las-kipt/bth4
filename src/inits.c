#include <zephyr.h>
#include <kernel.h>
#include <drivers/adc.h>
#include <drivers/gpio.h>
#include <init.h>


#ifdef DT_INST_0_PWR_EN_GPIO_GPIOS_CONTROLLER

static int pwr_en_gpio_init(struct device * dptr){
     (void) dptr;
     struct device * pwr_en_gpio_dev = device_get_binding(DT_INST_0_PWR_EN_GPIO_GPIOS_CONTROLLER);
     if (pwr_en_gpio_dev){
        gpio_pin_configure(pwr_en_gpio_dev, DT_INST_0_PWR_EN_GPIO_GPIOS_PIN, 
              GPIO_OUTPUT | DT_PWR_EN_GPIO_SENSOR_POWER_GPIO_GPIOS_FLAGS);
        gpio_pin_set(pwr_en_gpio_dev, DT_INST_0_PWR_EN_GPIO_GPIOS_PIN, 1);
     }
     return 0;
}

// Power enable GPIO init should occur after GPIO but before drivers
SYS_INIT(pwr_en_gpio_init, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE);
#endif