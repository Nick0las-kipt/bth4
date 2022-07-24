#include <zephyr.h>
#include <device.h>
#include <drivers/gpio.h>
#include "buttons.h"

#define LOG_LEVEL 3
#include <logging/log.h>
LOG_MODULE_REGISTER(buttons);

typedef struct button_pin_s {
    uint32_t pin;
    uint32_t flags;

} button_pin_t;

static button_pin_t buttons_matrix[] = {
#ifdef  DT_GPIO_KEYS_BUTTON_BACK_GPIOS_PIN
    { DT_GPIO_KEYS_BUTTON_BACK_GPIOS_PIN,
        DT_GPIO_KEYS_BUTTON_BACK_GPIOS_FLAGS
    },
#endif
#ifdef  DT_GPIO_KEYS_BUTTON_ENTER_GPIOS_PIN
    { DT_GPIO_KEYS_BUTTON_ENTER_GPIOS_PIN,
        DT_GPIO_KEYS_BUTTON_ENTER_GPIOS_FLAGS
    },
#endif
#ifdef  DT_GPIO_KEYS_BUTTON_UP_GPIOS_PIN
    { DT_GPIO_KEYS_BUTTON_UP_GPIOS_PIN,
        DT_GPIO_KEYS_BUTTON_UP_GPIOS_FLAGS
    },
#endif
#ifdef  DT_GPIO_KEYS_BUTTON_DOWN_GPIOS_PIN
    { DT_GPIO_KEYS_BUTTON_DOWN_GPIOS_PIN,
        DT_GPIO_KEYS_BUTTON_DOWN_GPIOS_FLAGS
    },
#endif
};

#define NUM_BUTTONS (sizeof(buttons_matrix)/sizeof(buttons_matrix[0]))

static struct device *gpio_buttons;
static uint32_t saved_pin_status;
static uint32_t buttons_invert_mask = 0;
static k_tid_t button_wake_tid = NULL;

typedef uint32_t key_msg_t;

K_MSGQ_DEFINE(buttons_msgq, sizeof(key_msg_t), 16, 4);

uint32_t button_cntrs[NUM_BUTTONS];

void get_button_status(int32_t *button_num, int32_t *bcntr){
    int32_t rnum = -1;
    int32_t rval = 0x7fffffff;
    for (int button = 0; button < NUM_BUTTONS; ++button){
        if (button_cntrs[button] && (button_cntrs[button] < rval)){
            rnum = button;
            rval = button_cntrs[button];
        }
    }
    *button_num = rnum;
    *bcntr = rval;    
}

uint32_t process_button_status(int32_t *button_num, int32_t *bcntr, uint32_t nevents){
static key_msg_t old_event;
    uint32_t cntr = 0;
    uint32_t rcntr = 0;
    do {
        for (int button = 0; button < NUM_BUTTONS; ++button){
            if (button_cntrs[button]) ++button_cntrs[button];
        }
        ++cntr;
        if (cntr > nevents) break;

        key_msg_t new_event;
        if (0 == k_msgq_get(&buttons_msgq, &new_event, K_NO_WAIT)){
            LOG_DBG("Get event 0x%x", new_event);
            new_event ^= buttons_invert_mask;
            for (int button = 0; button < NUM_BUTTONS; ++button){
                uint32_t mask = BIT(button);
                uint32_t old_m = mask & old_event;
                uint32_t new_m = mask & new_event;
                if (new_m ^ old_m) {
                    button_cntrs[button] = (new_m) ? 1 : 0;
                }
            }
            ++rcntr;
            old_event = new_event;
        }
    } while (cntr < nevents);
    get_button_status(button_num, bcntr);
    return rcntr;
}

static uint32_t get_pin_status(){
    uint32_t rv = 0;
    int val;
    if (gpio_buttons){
        for (int button = 0; button < NUM_BUTTONS; ++button){
            val = gpio_pin_get(gpio_buttons, buttons_matrix[button].pin);
            if (val > 0) rv |= BIT(button);
        }
    }
    return rv;
}

void buttons_timer_handler(struct k_timer *dummy)
{
    if (get_pin_status() == saved_pin_status) {
        LOG_DBG("++ Match 0x%x",saved_pin_status);
        if (k_msgq_put(&buttons_msgq, &saved_pin_status, K_NO_WAIT)){
            LOG_INF("Could not queue key event 0x%x",saved_pin_status);
        }
        if (button_wake_tid) k_wakeup(button_wake_tid);
    }
}

void buttons_set_listener(k_tid_t _wake_tid){
    button_wake_tid = _wake_tid;
}

K_TIMER_DEFINE(buttons_timer, buttons_timer_handler, NULL);

void button_event(struct device *gpiob, struct gpio_callback *cb,
            uint32_t pin)
{
    LOG_DBG("Trigger pin %x", pin);
    saved_pin_status = get_pin_status();
    k_timer_start(&buttons_timer, K_MSEC(2), K_MSEC(0));
}

static struct gpio_callback gpio_cb;

void buttons_init(void)
{
    gpio_buttons = device_get_binding(DT_GPIO_KEYS_BUTTON_BACK_GPIOS_CONTROLLER);

    if (!gpio_buttons) {
        LOG_ERR("Could not find controller \"%s\"", DT_GPIO_KEYS_BUTTON_BACK_GPIOS_CONTROLLER);
        return;
    }
    uint32_t mask = 0;
    for (int button = 0; button < NUM_BUTTONS; ++button){
        int rv = gpio_pin_configure(gpio_buttons, buttons_matrix[button].pin, 
            GPIO_INPUT | buttons_matrix[button].flags);
        if (rv < 0) LOG_ERR("Could not configure button #%d (pin %d)", button, buttons_matrix[button].pin);
        rv = gpio_pin_interrupt_configure(gpio_buttons, buttons_matrix[button].pin,
             GPIO_INT_EDGE_BOTH);
        if (rv < 0) LOG_ERR("Could not configure button #%d (pin %d)", button, buttons_matrix[button].pin);
        if (GPIO_PULL_UP & buttons_matrix[button].flags) buttons_invert_mask |= BIT(button);
        mask |= BIT(buttons_matrix[button].pin);
    }

    gpio_init_callback(&gpio_cb, button_event, mask);
    int rv = gpio_add_callback(gpio_buttons, &gpio_cb);
    if (rv < 0) LOG_ERR("Could not configure buttons interrupt (mask 0x%x)", mask);
    /*for (int button = 0; button < NUM_BUTTONS; ++button){
        gpio_pin_enable_callback(gpio_buttons, buttons_matrix[button].pin);
    }*/
}
