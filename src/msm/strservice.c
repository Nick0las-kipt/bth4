#include "strservice.h"

char *itoSPadded(int32_t val, char *str, uint8_t bytes, uint8_t decimalpos)  {
  uint8_t neg = 0;
  if(val < 0) {
    neg = 1;
    val = -val;
  }

  str[bytes] = 0;
  for(;;) {
    if(bytes == decimalpos) {
      str[--bytes] = DECIMAL;
    if (0 == bytes) break;
      decimalpos = 0;
    }
    str[--bytes] = '0' + (val % 10);
    val = val / 10;
    if(bytes == 0 || (decimalpos == 0 && val == 0))
      break;
  }

  if(neg && bytes > 0)
    str[--bytes] = '-';

  while(bytes != 0)
    str[--bytes] = ' ';
  return str;
}

uint8_t strLen(char *str){
 uint8_t i=0;
 while((str[i])&&(i<255))i++;
 return i;
}

uint8_t strFix(char *str,uint8_t newlen){
 uint8_t i=0;
 while((str[i])&&(i<newlen))i++;
 uint8_t j=i;
 while(i<newlen) {
   str[i]=' ';
   i++;
 }
 str[newlen]=0;
 return i-j;
}

uint8_t strCpyL(char *dst,const char *src,uint8_t maxlen){
  uint8_t i=0;
  char c;
  do{
    c=*src++;
    *dst++=c;
    ++i;
  }while ((i<maxlen)&&(c));
  if (i >= maxlen) dst[maxlen] = 0;
  return i;
}

