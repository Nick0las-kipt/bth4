#include <zephyr.h>
#include <sys/printk.h>
#include <kernel.h>
#include <drivers/spi.h>
#include <drivers/gpio.h>
#include <drivers/pwm.h>

#include "lpm013m126.h"

#define LOG_LEVEL 4
#include <logging/log.h>
#include "backlight.h"

LOG_MODULE_REGISTER(mlcd);


static struct device * lcd_cs_gpio_dev;
static struct device * lcd_disp_gpio_dev;
static struct device * lcd_spi_dev;

static int lcd_cs_pin = -1;
static int lcd_extcom_pin = -1;
static int lcd_disp_pin = -1;

static struct spi_config lcd_spi_cfg = {
    .frequency = DT_INST_0_JDI_LPM013M126_SPI_MAX_FREQUENCY,
    .operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB,
};

#ifdef DT_INST_0_JDI_LPM013M126_PWMS_CONTROLLER
#define _INST_0_JDI_LPM013M126_EXTCOM_PWM
static struct device * lcd_extcom_pwm_dev;
#else // DT_INST_0_JDI_LPM013M126_PWMS_CONTROLLER

#define _INST_0_JDI_LPM013M126_EXTCOM_THREAD

#define LCD_EXTCOM_PRIORITY          0
#define LCD_EXTCOM_STACKSIZE         400

K_THREAD_STACK_DEFINE(lcd_extcom_stack, LCD_EXTCOM_STACKSIZE);
struct k_thread lcd_extcom_thread_data;
k_tid_t lcd_extcom_tid;

#ifdef DT_INST_0_JDI_LPM013M126_EXTCOM_GPIOS_CONTROLLER
#define _INST_0_JDI_LPM013M126_EXTCOM_GPIO
static struct device * lcd_extcom_gpio_dev;
#else
// #define _INST_0_JDI_LPM013M126_EXTCOM_SPI
#error "No extcom device"
#endif // DT_INST_0_JDI_LPM013M126_DISP_GPIOS_CONTROLLER
#endif // DT_INST_0_JDI_LPM013M126_PWMS_CONTROLLER

int lcd_spi_xfer_simple(void * buf, size_t count){
    int rv = 0;
    gpio_pin_set(lcd_cs_gpio_dev, lcd_cs_pin, 1);
    k_busy_wait(6);
    const struct spi_buf w_spi_buf={buf, count};
    const struct spi_buf_set w_spi_set = {&w_spi_buf, 1};
    if (lcd_spi_dev) rv = spi_write(lcd_spi_dev, &lcd_spi_cfg, &w_spi_set);
    k_busy_wait(2);
    gpio_pin_set(lcd_cs_gpio_dev, lcd_cs_pin, 0);
    return rv;
}

int lcd_spi_xfer_line(void * buf, int line, int bpp){
    int bytes_in_line = 0;
    uint8_t cmd[2];
    int rv = 0;
    switch (bpp){
        case 4:
            bytes_in_line = 90;
            cmd[0] = CMD_SINGLE_LINE_4_BIT;
            break;
        case 3:
            bytes_in_line = 61;
            cmd[0] = CMD_SINGLE_LINE_3_BIT;
            break;
        case 1:
            bytes_in_line = 24;
            cmd[0] = CMD_SINGLE_LINE_1_BIT;
            break;
        default:
            return -EINVAL;
    }    
    cmd[1] = 1 + line;

    gpio_pin_set(lcd_cs_gpio_dev, lcd_cs_pin, 1);
    k_busy_wait(6);
 
    const struct spi_buf w_spi_buf[2] = {{cmd, 2}, {buf, bytes_in_line} };
    const struct spi_buf_set w_spi_set = {(struct spi_buf *) &w_spi_buf, 2};
    if (lcd_spi_dev) rv = spi_write(lcd_spi_dev, &lcd_spi_cfg, &w_spi_set);

    k_busy_wait(2);
    gpio_pin_set(lcd_cs_gpio_dev, lcd_cs_pin, 0);
    return rv;
    
}

static struct k_poll_signal s_signal;

int lcd_spi_xfer_line_async(void * buf, int line, int bpp){
    int bytes_in_line = 0;
    static uint8_t cmd[2];
    int rv = 0;
    switch (bpp){
        case 4:
            bytes_in_line = 90;
            cmd[0] = CMD_SINGLE_LINE_4_BIT;
            break;
        case 3:
            bytes_in_line = 61;
            cmd[0] = CMD_SINGLE_LINE_3_BIT;
            break;
        case 1:
            bytes_in_line = 24;
            cmd[0] = CMD_SINGLE_LINE_1_BIT;
            break;
        default:
            return -EINVAL;
    }    
    cmd[1] = 1 + line;

    k_poll_signal_init(&s_signal);

    gpio_pin_set(lcd_cs_gpio_dev, lcd_cs_pin, 1);
    k_busy_wait(6);

    static struct spi_buf w_spi_buf[2] = {{cmd, 2}, {0, 0} };
    static struct spi_buf_set w_spi_set = {(struct spi_buf *) &w_spi_buf, 2};

    w_spi_buf[1].buf = buf;
    w_spi_buf[1].len = bytes_in_line;
    
    if (lcd_spi_dev)
        rv = spi_transceive_async(lcd_spi_dev, &lcd_spi_cfg, &w_spi_set, NULL, &s_signal);
    else
        rv = -EINVAL;
    return rv;
}

int lcd_wait_async(){
    struct k_poll_event events[1]={
        K_POLL_EVENT_INITIALIZER(K_POLL_TYPE_SIGNAL, K_POLL_MODE_NOTIFY_ONLY, &s_signal),
    };
    k_poll(events, 1, K_FOREVER);
    k_busy_wait(2);
    gpio_pin_set(lcd_cs_gpio_dev, lcd_cs_pin, 0);
    return s_signal.result;
}

int lcd_disoff(){
    int ret;
    lcd_disp_gpio_dev = device_get_binding(DT_INST_0_JDI_LPM013M126_DISP_GPIOS_CONTROLLER);
    if (!(lcd_disp_gpio_dev)){
        LOG_ERR("LCD GPIO not found!");
        return -ENODEV;
    }
    lcd_disp_pin = DT_INST_0_JDI_LPM013M126_DISP_GPIOS_PIN;
    if ((ret = gpio_pin_configure(lcd_disp_gpio_dev, lcd_disp_pin, GPIO_OUTPUT)) < 0){
        LOG_ERR("DISP pin config error %d!", ret);
        return ret;
    }
    if ((ret = gpio_pin_set(lcd_disp_gpio_dev, lcd_disp_pin, 0)) < 0){
        LOG_ERR("DISP pin config error %d!", ret);
        return ret;
    }
    return 0;
}

#ifdef _INST_0_JDI_LPM013M126_EXTCOM_THREAD

static int extcom_freq = LCD_EXTCOM_FMIN;

static void bl_event(int on){
    extcom_freq = on ? LCD_EXTCOM_FMAX : LCD_EXTCOM_FMIN;
}

static void lcd_extcom_entry(void* p1, void * p2, void * p3){
    (void) p1;
    (void) p2;
    (void) p3;
    lcd_bl_set_onoff_event(bl_event);
    while (1){
        gpio_pin_set(lcd_extcom_gpio_dev, lcd_extcom_pin, 1);
        k_sleep(K_MSEC(LCD_EXTCOM_T_US/1000));
        gpio_pin_set(lcd_extcom_gpio_dev, lcd_extcom_pin, 0);
        int delay_ms = (1000/extcom_freq) - (LCD_EXTCOM_T_US/1000);
        k_sleep(K_MSEC(delay_ms));
    }
}
#endif

int lcd_init()
{
    int ret;
    uint8_t cmd[92];
    lcd_cs_gpio_dev = device_get_binding(DT_INST_0_JDI_LPM013M126_CS_GPIOS_CONTROLLER);
    if (!(lcd_cs_gpio_dev)){
        LOG_ERR("LCD CS GPIO not found!");
        return -ENODEV;
    }
    lcd_disp_gpio_dev = device_get_binding(DT_INST_0_JDI_LPM013M126_DISP_GPIOS_CONTROLLER);
    if (!(lcd_disp_gpio_dev)){
        LOG_ERR("LCD DSIP GPIO not found!");
        return -ENODEV;
    }
#ifdef DT_INST_0_JDI_LPM013M126_PWMS_CONTROLLER
    lcd_extcom_pwm_dev = device_get_binding(DT_INST_0_JDI_LPM013M126_PWMS_CONTROLLER);
    if (!(lcd_extcom_pwm_dev)){
        LOG_ERR("LCD EXTCOM PWM not found!");
        return -ENODEV;
    }
#endif

    lcd_spi_dev = device_get_binding(DT_INST_0_JDI_LPM013M126_BUS_NAME);
    if (!(lcd_spi_dev)){
        LOG_ERR("LCD SPI not found!\n");
        return -ENODEV;
    }

    lcd_cs_pin = DT_INST_0_JDI_LPM013M126_CS_GPIOS_PIN;
    if ((ret = gpio_pin_configure(lcd_cs_gpio_dev, lcd_cs_pin, GPIO_OUTPUT)) < 0){
        LOG_ERR("CS pin config error %d!", ret);
        return ret;
    }
    if ((ret = gpio_pin_set(lcd_cs_gpio_dev, lcd_cs_pin, 0)) < 0){
        LOG_ERR("CS pin config error %d!", ret);
        return ret;
    }
    lcd_disp_pin = DT_INST_0_JDI_LPM013M126_DISP_GPIOS_PIN;
    if ((ret = gpio_pin_configure(lcd_disp_gpio_dev, lcd_disp_pin, GPIO_OUTPUT)) < 0){
        LOG_ERR("DISP pin config error %d!", ret);
        return ret;
    }
    if ((ret = gpio_pin_set(lcd_disp_gpio_dev, lcd_disp_pin, 0)) < 0){
        LOG_ERR("DISP pin write error %d!", ret);
        return ret;
    }
#ifdef DT_INST_0_JDI_LPM013M126_PWMS_CHANNEL    
    lcd_extcom_pin = DT_INST_0_JDI_LPM013M126_PWMS_CHANNEL;
#endif
    /*if ((ret = gpio_pin_configure(lcd_disp_gpio_dev, lcd_extcom_pin, GPIO_OUTPUT)) < 0){
        LOG_ERR("EXTCOMIN pin config error %d!", ret);
        return ret;
    }*/
#ifdef _INST_0_JDI_LPM013M126_EXTCOM_PWM
    if ((!lcd_extcom_pwm_dev) || (lcd_extcom_pin < 0) || 
        (ret = pwm_pin_set_usec(lcd_extcom_pwm_dev, lcd_extcom_pin, (USEC_PER_SEC / LCD_EXTCOM_F), LCD_EXTCOM_T_US)) < 0){
        gpio_pin_set(lcd_disp_gpio_dev, lcd_disp_pin, 0);
        LOG_ERR("Extcomin PWM failed, turning off display");
        return ret;
    }
#endif // _INST_0_JDI_LPM013M126_EXTCOM_PWM
#ifdef DT_INST_0_JDI_LPM013M126_EXTCOM_GPIOS_CONTROLLER
    lcd_extcom_gpio_dev = device_get_binding(DT_INST_0_JDI_LPM013M126_EXTCOM_GPIOS_CONTROLLER);
    if (!(lcd_extcom_gpio_dev)){
        LOG_ERR("LCD EXTCOM GPIO not found!");
        return -ENODEV;
    }
    lcd_extcom_pin = DT_INST_0_JDI_LPM013M126_EXTCOM_GPIOS_PIN;
    if ((ret = gpio_pin_configure(lcd_extcom_gpio_dev, lcd_extcom_pin, GPIO_OUTPUT)) < 0){
        LOG_ERR("EXTCOM pin config error %d!", ret);
        return ret;
    }
    if ((ret = gpio_pin_set(lcd_extcom_gpio_dev, lcd_extcom_pin, 0)) < 0){
        LOG_ERR("EXTCOM pin write error %d!", ret);
        return ret;
    }
#endif
#ifdef _INST_0_JDI_LPM013M126_EXTCOM_THREAD
    LOG_WRN("Starting EXTCOM thread");
    lcd_extcom_tid = k_thread_create(&lcd_extcom_thread_data, lcd_extcom_stack,
                             K_THREAD_STACK_SIZEOF(lcd_extcom_stack),
                             lcd_extcom_entry,
                             NULL, NULL, NULL,
                             LCD_EXTCOM_PRIORITY, 0, K_NO_WAIT);
    if (lcd_extcom_tid) k_thread_name_set(lcd_extcom_tid, "LCD extcom");
#endif

    cmd[1] = 0;
    cmd[0] = CMD_CLEAR_ALL;
    ret = lcd_spi_xfer_simple(&cmd, 2);
    if (ret < 0) return ret;
    k_busy_wait(2000);
    gpio_pin_set(lcd_disp_gpio_dev, lcd_disp_pin, 1);
    k_busy_wait(2000);
    for(int line = 1; line <=176; ++line){
        for(int i = 0; i < 88; ++i){
            int num = ((i >> 2) + (line >> 2)) & 0x7;
            cmd[i+2] = (num << 5) + (num << 1);
        }
        cmd[0] = CMD_SINGLE_LINE_4_BIT;
        cmd[1] = line;
        lcd_spi_xfer_simple(&cmd, 92);
    }
    return 0;
}

int lcd_clear(){
    uint8_t cmd[2];
    cmd[1] = 0;
    cmd[0] = CMD_CLEAR_ALL;
    return lcd_spi_xfer_simple(&cmd, 2);    
}

int lcd_blink_off(){
    uint8_t cmd[2];
    cmd[1] = 0;
    cmd[0] = CMD_BLINK_OFF;
    return lcd_spi_xfer_simple(&cmd, 2);    
}