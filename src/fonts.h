#ifndef _FONTS_H
#define _FONTS_H

#include "stdint.h"
#include "ui-config.h"

#if (USE_LARGER_FONT)
#define LCHAR_H 16
#define LCHAR_W 12
#endif

#define CHAR_H 14
#define CHAR_W 8

#if (USE_LARGER_FONT)
#define CCHAR_H LCHAR_H
#define CCHAR_W LCHAR_W
#else
#define CCHAR_H CHAR_H
#define CCHAR_W CHAR_W
#endif

#define COL2(fcol,bcol) ((fcol + (bcol << 4)) << 1)

#ifdef __cplusplus
extern "C" {
#endif

#if (USE_LARGER_FONT)
void char_to_fb_4bit_pl_larger(uint8_t * ptr, char c, int startl, int endl, uint32_t fc, uint32_t bc, uint32_t line_delta);
void put_str_col_extbuf_larger(uint8_t * buffer, uint8_t x, uint8_t line, const char *str, const uint8_t * colstr, uint8_t length);
void print_str_col_extbuf_larger(uint8_t * buffer, uint8_t x, uint8_t y, const char *str, const uint8_t * colstr, uint8_t length);
#endif

void char_to_fb_4bit_pl(uint8_t * ptr, char c, int startl, int endl, uint32_t fc, uint32_t bc, uint32_t line_delta);
void arr_to_fb_4bit_pl(uint8_t * ptr, const char * str, int length, int startl, int endl, uint32_t fc, uint32_t bc, uint32_t line_delta);

void print_str_col_extbuf(uint8_t * buffer, uint8_t x, uint8_t y, const char *str, const uint8_t * colstr, uint8_t length);
void put_str_col_extbuf(uint8_t * buffer, uint8_t x, uint8_t line, const char *str, const uint8_t * colstr, uint8_t length);

#if (USE_LARGER_FONT)
#define PRINT_STR_COL_EXTBUF_CL print_str_col_extbuf_larger
#define PUT_STR_COL_EXTBUF_CL put_str_col_extbuf_larger
#else
#define PRINT_STR_COL_EXTBUF_CL print_str_col_extbuf
#define PUT_STR_COL_EXTBUF_CL put_str_col_extbuf
#endif


#ifdef __cplusplus
}
#endif

#endif