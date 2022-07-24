#include <zephyr.h>
#include <sys/printk.h>
#include <kernel.h>
#include <drivers/gpio.h>
#include <drivers/pwm.h>

#include "backlight.h"

#define LOG_LEVEL 2
#include <logging/log.h>
LOG_MODULE_REGISTER(backlight);

#ifdef DT_INST_0_LED_LCD_BL

#define BL_PRIORITY          0
#define BL_STACKSIZE         512

static struct device * lcd_bl_gpio_dev;
static struct device * lcd_bl_pwm_dev;
static int lcd_bl_pin = -1;

static uint32_t lcd_deadline = 0;
static uint32_t lcd_delay_ticks;
static int lcd_fadeout_period = FADE_TIME_MS;
static uint8_t g_bl_on_flag = 0;
static uint8_t g_bl_enable_flag = 1;

static void (*bl_on_off_event)(int) = NULL;

void lcd_bl_set_onoff_event(void(*event)(int)){
    bl_on_off_event = event;
}

static int32_t bl_ms_left(void){
    uint32_t left32 = lcd_deadline - k_cycle_get_32();
    if (left32 < 0x7fffffff) return (1000U * left32)/sys_clock_hw_cycles_per_sec();
    lcd_deadline = k_cycle_get_32() - 1;
    return -1;
}

static inline void lcd_bl_on(){
    if ((bl_on_off_event) && (!g_bl_on_flag)) bl_on_off_event(1);
    g_bl_on_flag = 1;
    if (lcd_bl_pin < 0) return;
    if (lcd_bl_pwm_dev) {
        pwm_pin_set_usec(lcd_bl_pwm_dev, lcd_bl_pin, FADE_PERIOD_US, FADE_PERIOD_US, 0);
        LOG_DBG("BL is on: PWM");
    } else {
        gpio_pin_set(lcd_bl_gpio_dev, lcd_bl_pin, 1);
        LOG_DBG("BL is on: GPIO");
    }
}

static inline void lcd_bl_off(){
    if ((bl_on_off_event) && (g_bl_on_flag)) bl_on_off_event(0);
    g_bl_on_flag = 0;
    if (lcd_bl_pin < 0) return;
    if (lcd_bl_pwm_dev) {
        pwm_pin_set_usec(lcd_bl_pwm_dev, lcd_bl_pin, FADE_PERIOD_US, 0, 0);
        LOG_DBG("BL is off: PWM");
    } else {
        gpio_pin_set(lcd_bl_gpio_dev, lcd_bl_pin, 0);   
        LOG_DBG("BL is off: GPIO");
    }
}

static void lcd_bl_entry(void* p1, void * p2, void * p3){
    (void) p1;
    (void) p2;
    (void) p3;
    while (1){
        int left_ms;
        while ((left_ms = bl_ms_left()) > 0){
            int time_to_sleep = (lcd_bl_pwm_dev) ? left_ms - lcd_fadeout_period : left_ms;
            LOG_DBG("BL is on, %d ms left",left_ms);
            if ((lcd_bl_pwm_dev) && (left_ms < lcd_fadeout_period)) {
                time_to_sleep = FADE_STEP_MS;
                int pulse_us = (FADE_PERIOD_US * left_ms)/ lcd_fadeout_period;
                pwm_pin_set_usec(lcd_bl_pwm_dev, lcd_bl_pin, FADE_PERIOD_US, pulse_us, 0);
            } else {
                lcd_bl_on();
            }
            k_sleep(K_MSEC(time_to_sleep));
        }
        LOG_DBG("Turning BL off");
        lcd_bl_off();
        k_sleep(K_MINUTES(10));
    }
}

void lcd_bl_set_timeout_ms(unsigned int ms){
    lcd_delay_ticks = (sys_clock_hw_cycles_per_sec() * ms)/1000U;
    LOG_DBG("BL timeout is set to %d ms", ms);
}

unsigned int lcd_bl_get_timeout_ms(){
    unsigned int div = sys_clock_hw_cycles_per_sec();
    if (0 == div) div = 1;
    return lcd_delay_ticks * 1000U / div;
}

K_THREAD_STACK_DEFINE(lcd_bl_stack, BL_STACKSIZE);
struct k_thread lcd_bl_thread_data;
k_tid_t lcd_bl_tid;

void lcd_bl_prolongate(int force){
    if (!g_bl_enable_flag) return;
    if (force || (bl_ms_left() >= 0)){
        lcd_deadline = lcd_delay_ticks + k_cycle_get_32();
        LOG_DBG("Prolongate BL, deadline updated, force = %d", force);
    } else {
        LOG_DBG("Prolongate BL, no deadline update (BL is off)");
    }
    if (lcd_bl_tid) k_wakeup(lcd_bl_tid);
}

int lcd_bl_enable_get(){
    return g_bl_enable_flag;
}

void lcd_bl_enable_set(int enable){
    g_bl_enable_flag = enable;
    if (!enable){
        uint32_t new_deadline = k_cycle_get_32();
        if (lcd_bl_pwm_dev) new_deadline += lcd_fadeout_period;
        lcd_deadline = new_deadline;
        if (lcd_bl_tid) k_wakeup(lcd_bl_tid);
    }
}

int lcd_is_bl_on(){
    return g_bl_on_flag;
}

void lcd_bl_init(void)
{
#ifdef DT_LED_LCD_BL_BACKLIGHT_GPIOS_CONTROLLER    
    lcd_bl_gpio_dev = device_get_binding(DT_LED_LCD_BL_BACKLIGHT_GPIOS_CONTROLLER);
#endif
    if (!(lcd_bl_gpio_dev)){
        LOG_INF("LCD BL GPIO not found!");
    } else {
#ifdef DT_LED_LCD_BL_BACKLIGHT_GPIOS_PIN        
        lcd_bl_pin = DT_LED_LCD_BL_BACKLIGHT_GPIOS_PIN;
#endif 
        int ret;
        if ((ret = gpio_pin_configure(lcd_bl_gpio_dev, lcd_bl_pin, GPIO_OUTPUT)) < 0){
            LOG_ERR("BL pin config error %d!", ret);
            return;
        }           
    }
#ifdef DT_LED_LCD_BL_BACKLIGHT_PWMS_CONTROLLER
    lcd_bl_pwm_dev = device_get_binding(DT_LED_LCD_BL_BACKLIGHT_PWMS_CONTROLLER);
#endif
    if (!(lcd_bl_pwm_dev)){
        LOG_INF("LCD BL PWM not found!");
    } else {
#ifdef DT_LED_LCD_BL_BACKLIGHT_PWMS_CHANNEL        
        lcd_bl_pin = DT_LED_LCD_BL_BACKLIGHT_PWMS_CHANNEL;
#endif        
    }

    lcd_bl_set_timeout_ms(DEFAULT_BL_TIMEOUT_MS);
    
    if ( (!lcd_bl_tid) && ((lcd_bl_gpio_dev) || (lcd_bl_pwm_dev))) {
        lcd_bl_tid = k_thread_create(&lcd_bl_thread_data, lcd_bl_stack,
                                 K_THREAD_STACK_SIZEOF(lcd_bl_stack),
                                 lcd_bl_entry,
                                 NULL, NULL, NULL,
                                 BL_PRIORITY, 0, K_NO_WAIT);
        if (lcd_bl_tid) k_thread_name_set(lcd_bl_tid, "LCD bl ctl");
    } else {
        LOG_ERR("COULD not locate BL GPIO or PWM device");
    }
}
#endif