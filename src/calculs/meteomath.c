#include "calculs/meteomath.h"

static inline float fast_log (float val)
{
//     uint32_t*         exp_ptr = reinterpret_cast < uint32_t*>(&val);
/*     int32_t x = (uint32_t&) val;
     int32_t log_2 = ((x >> 23) & 255) - 128;
     x &= ~(255L << 23);
     x += 127L << 23;
     float fval = (float&)x; */
     union {int32_t i; float f;} x;
     x.f = val;
     int32_t log_2 = ((x.i >> 23) & 255) - 128;
     x.i &= ~(255L << 23);
     x.i += 127L << 23;
     float fval = x.f;


    fval = ((-1.0f/3) * fval + 2) * fval - 2.0f/3;

    return (fval + log_2)*0.693f;
}

inline float fast_exp(float val){
     union {int32_t i; float f;} x;
     x.f = val/0.693f;
     int32_t int_part = floorf(x.f);
     float frac_part = x.f - int_part;
     x.f = 1.0f + frac_part * (0.656f + frac_part * 0.344f);
     uint8_t log_2 = ((x.i >> 23) & 255);     
     x.i &= ~(255L << 23);
     x.i += (((log_2 + int_part) & 255) << 23);
     return x.f;        
}

int16_t getDewPoint(int16_t temp, uint8_t RH){    // temp in 0.01 deg C, RH in percent
    uint32_t lambda;
    float beta;
/*    if (temp>0){
    }else{
        lambda=27262;
        beta=22.46f;
    }*/
    //we do not use "above ice" model
        lambda = 24312;
        beta = 17.62f;
    float LRH;
    if (RH){
        LRH = fast_log(RH/100.0f);
    } else {
        LRH = -5.0f; //Just very low value 
    }
    float Tdel = (beta*temp)/(lambda+temp);
    float TDO = lambda*(LRH+Tdel)/(beta-LRH-Tdel);
    return TDO;
}

uint8_t getRHfromDewTemp(int16_t temp, int16_t TDI){
    uint32_t lambda;
    float beta;
    lambda = 24312;
    beta = 17.62f;
    float Tdel=(beta*temp)/(lambda+temp);
    float k=((float) TDI)/((float) lambda);
    float LRH = (beta*k - Tdel*(1.0f + k))/(1.0f + k);
    uint8_t RH = (100.0f*fast_exp(LRH)+0.5);
    return (RH > 100) ? 100 : RH;    
}


int16_t getBaroAlt(uint32_t P, uint32_t pRef, int16_t T){
    // baro ratio for T=288k for the first time
    uint32_t Alr_ratio = 8435L;
    float    Alt;
    int16_t OutAlt;
#if defined(ALT_WITH_TEMP)
    Alr_ratio=(Alr_ratio*(T+27315L)/28815L); 
#endif
    // P expected in MMHG*100
    if (P <= 20000) P = 20000;
    if(P > pRef){
        Alt = (float)pRef / (float)P;
    }else{
        Alt = (float)P / (float)pRef;
    }
    Alt = (fast_log(Alt) * Alr_ratio);
    OutAlt = (int16_t) (Alt - 0.5f);
    if (P <= pRef) OutAlt = -OutAlt;
    return OutAlt;    
}

uint32_t getPressureFromAlt(int16_t alt, uint32_t pRef, int16_t T){
    uint32_t Alr_ratio = 8435L;
    float    fAlt,fP;
#if defined(ALT_WITH_TEMP)
    Alr_ratio = (Alr_ratio * (T + 27315L) / 28815L); 
#endif
    // P expected in MMHG*100
    if (alt >= 0) fAlt=alt; else fAlt=-alt;
    fAlt = fast_exp(fAlt/Alr_ratio);
    if (alt >= 0) fP = pRef / fAlt; else fP = pRef * fAlt;
    return (0.5f + fP);
}

colorxy_t getUVfromXY(colorxy_t colXY){
  colorxy_t colUV;
  float denom=3.0f-2.0f*colXY.x+12.f*colXY.y;
  colUV.x=4.0f*colXY.x/denom;
  colUV.y=6.0f*colXY.y/denom;
  return colUV;
}

colorxy_t getXYfromUV(colorxy_t colUV){
  colorxy_t colXY;
  float denom=(8.0f+4.0f*colUV.x-16.0f*colUV.y);
  colXY.x=6.0f*colUV.x/denom;
  colXY.y=4.0f*colUV.y/denom;
  return colXY;
}


colorxy_t getUVfromColorTemp(float t){
  colorxy_t colUV;
  colUV.x=(0.860117757f+1.54118254e-4*t+1.28641212e-7*t*t)/(1.0f+8.42420235e-4*t+7.08145163e-7*t*t);
  colUV.y=(0.317398726f+4.22806245e-5*t+4.20481691e-8*t*t)/(1.0f-2.89741816e-5*t+1.61456053e-7*t*t);
  return colUV;
}

#define Xe 0.3320f
#define Ye 0.1858f

float getColorTemp(colorxy_t color){
  float outf,fn;
  if ((color.y-Ye)>0.0001f){
    fn = (color.x-Xe)/(Ye-color.y);
    if (fn<-1.22f) fn=-1.22f; // constraint for low temp
    outf = 5520.0f+fn*(6823.3f+fn*(3525.0f+fn*449.0f));
  } else outf=99999.0f;
  return outf;
}