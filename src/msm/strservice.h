#ifndef _STRSERVICE_H
#define _STRSERVICE_H

#include <inttypes.h>

#ifndef DECIMAL
#define DECIMAL '.'
#endif
#ifdef __cplusplus
extern "C" {
#endif
char *itoSPadded(int32_t val, char *str, uint8_t bytes, uint8_t decimalpos);
// Converts integer val and outputs to string *str
// bytes - exact length of output
// decimalpos-position to draw decimal symbol
// decimalpos-bytes- last
// decimalpos=0 conversion with no decimal 
// decimalpos>bytes - conversion with no decimal and leading zeros

uint8_t strLen(char *str);
// get length of str

uint8_t strFix(char *str,uint8_t newlen);
// Fix length of str to newlen
// if shorter adds spaces
// if longer just chops
// returns number of spaces inserted

uint8_t strCpyL(char *dst,const char *src,uint8_t maxlen);
// copy strings in RAM limiting length to maxlen
// returns number of bytes copied 

#ifdef __cplusplus
}
#endif

inline uint8_t BCD2DEC(uint8_t arg){
  return (arg>>4)*10+(arg&0x0f);
}

inline uint8_t DEC2BCD(uint8_t arg){
  return ((arg/10)<<4)+(arg%10);
}

#endif
