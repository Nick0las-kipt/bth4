#ifndef _BACKLIGHT_H
#define _BACKLIGHT_H

#define DEFAULT_BL_TIMEOUT_MS 10000
#define FADE_TIME_MS           1000
#define FADE_STEP_MS             10
#define FADE_PERIOD_US         1000

#ifdef __cplusplus
extern "C" {
#endif

#ifdef DT_INST_0_LED_LCD_BL

void lcd_bl_set_timeout_ms(unsigned int ms);
unsigned int lcd_bl_get_timeout_ms();
void lcd_bl_prolongate(int force);
void lcd_bl_init(void);
void lcd_bl_set_onoff_event(void(*event)(int));
int lcd_is_bl_on(void);


#else // DT_INST_0_LED_LCD_BL
static inline void lcd_bl_set_timeout_ms(unsigned int ms){(void)ms;}
static inline unsigned int lcd_bl_get_timeout_ms() {return 0;}
static inline void lcd_bl_prolongate(int force){(void)force;}
static inline void lcd_bl_init(void){}
static inline void lcd_bl_set_onoff_event(void(*event)(int)){(void)event;}
static inline int lcd_is_bl_on(void){return 0;}
#endif

#ifdef __cplusplus
}
#endif

#endif