#ifndef _LPM013M126_H
#define _LPM013M126_H
#include <inttypes.h>
#include <stddef.h>

#define DISP_W                 (DT_INST_0_JDI_LPM013M126_WIDTH)
#define DISP_H                 (DT_INST_0_JDI_LPM013M126_HEIGHT)

#define LINE_BUF_L  (DISP_W/2 + 2)

#define CMD_SINGLE_LINE_3_BIT 0x80
#define CMD_SINGLE_LINE_4_BIT 0x90
#define CMD_SINGLE_LINE_1_BIT 0x88
#define CMD_CLEAR_ALL         0x20
#define CMD_BLINK_OFF         0xA0

#define LCD_EXTCOM_F             8
#define LCD_EXTCOM_FMAX          80
#define LCD_EXTCOM_FMIN          2

#define LCD_EXTCOM_T_US       10000

#ifdef __cplusplus
extern "C" {
#endif

int lcd_disoff(void);
int lcd_init();

int lcd_spi_xfer_simple(void * buf, size_t count);
int lcd_spi_xfer_line(void * buf, int line, int bpp);
int lcd_spi_xfer_line_async(void * buf, int line, int bpp);
int lcd_wait_async();

int lcd_clear();
int lcd_blink_off();

#ifdef __cplusplus
}
#endif

#endif