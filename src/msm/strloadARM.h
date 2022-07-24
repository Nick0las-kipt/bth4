#ifndef strloadARM_h
#define strloadARM_h
#include "strservice.h"
inline uint8_t pgmLoadStr(char *str, const char *sstr, uint8_t lLim){
    return strCpyL(str,sstr,lLim);
}

#endif
