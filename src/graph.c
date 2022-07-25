#include "graph.h"
#include "lpm013m126.h"
#include "fonts.h"
#include "ui-config.h"
#include "rtc.h"
#include "sensorlog.h"
#include "msm/msm-ids.h" // keys
#include "calculs/meteomath.h"
#include "dispservice.h"

#if UI_PRESENT
// **********************************
// * graph configuration parameters *
// **********************************

// Width of active graph part in pixels
// This is pos primary parameter, set depending on LCD size and layout you prefer
#ifndef GRAPH_WIDTH_PIX
  #define GRAPH_WIDTH_PIX  112
#endif
// Graph height in pixels equals to this * 10
#ifndef GRAPH_DEC_BLOCKS 
  #define GRAPH_DEC_BLOCKS 10
#endif
// Y scale tick length, 0-no tick
#ifndef GRAPH_Y_SCALE_TICK
  #define GRAPH_Y_SCALE_TICK 2
#endif
// X scale tick length, 0-no tick
#ifndef GRAPH_X_SCALE_TICK
  #define GRAPH_X_SCALE_TICK 2
#endif

#ifndef GRAPH_SCROLL_LIM
  #define GRAPH_SCROLL_LIM 2
#endif

#define LOG_LEVEL 4
#include <logging/log.h>
LOG_MODULE_REGISTER(graph);

typedef struct gr_tickdata_s {
    uint8_t gridDist; // geometrical grid lines distance in pixels
    uint8_t tickDist; // geometrical minor tick lines distance in pixels
    uint8_t labelDist; // geometrical Label and large tick distance in pixels

    uint8_t gridPhase;    // grid pixel count phase (marked at phase=0)
    uint8_t tickPhase;   // tick pixel count phase (marked at phase=0)
    uint8_t labelPhase;  // label pixel count phase (marked at phase=0)

    uint16_t gridPeriod;   // grid interval in X scale units
    uint16_t tickPeriod;   // minor tick interval in X scale units
    uint16_t labelPeriod;  // Label interval in X scale units
} gr_tickdata_t;

typedef struct gr_labeldata_s {
    int8_t  refHour;
    uint8_t leftDigs;
    uint8_t rightDigs;
    uint8_t middleIsBar;
    uint8_t leftRatio;
    uint8_t rightRatio;
    uint8_t leftDay;
    uint8_t leftMonth;
    uint8_t rightDay;
    uint8_t rightMonth;
    int scrollBefore;
    int scrollAfter;
    int scrollSize;
} gr_labeldata_t;

typedef struct gr_colorscheme_s {
    const uint8_t * graphColMatrix;
    uint8_t textFcol;
    uint8_t textBcol;
    uint8_t barBcol;
    uint8_t barLcol;
} gr_colorscheme_t;

static int16_t dataLeft[GRAPH_WIDTH_PIX];      // left scale data used for Temperature
static int16_t dataRight[GRAPH_WIDTH_PIX];     // middle scale data used for RH bar or dew temperature
static int16_t dataMiddle[GRAPH_WIDTH_PIX];    // rigth scale data used for Pressure

static int16_t fastGRT[GRAPH_WIDTH_PIX+1];      // fast mode Temperature data
static int16_t fastGRP[GRAPH_WIDTH_PIX+1];      // fast mode Pressure data
static int8_t fastGRRH[GRAPH_WIDTH_PIX+1];      // fast mode RH data
static int fastDataPos;

//TODO: Add this when we get stuff working
static uint32_t fastPRef;
static int16_t fastGRA[GRAPH_WIDTH_PIX+1];      // fast mode Alt data
static int16_t fastGRD[GRAPH_WIDTH_PIX+1];      // fast mode Dew Temperature data

static uint32_t last_shift_cycles_fast;
static int  fastUpdated;

static inline int graphPressAsAlt(int graphInterpretationMode){
    return graphInterpretationMode &1;
}

static inline int graphDewTemp(int graphInterpretationMode){
    return (graphInterpretationMode >> 1) & 1;
}

static void fixFastData(int graphInterpretationMode, uint32_t pRef){
    int i;
    if (fastPRef != pRef) {
        for (i=0;i<= GRAPH_WIDTH_PIX;++i){
            fastGRA[i] = -32767;
        }
        fastPRef = pRef;
    }
    if (graphPressAsAlt(graphInterpretationMode)){
        for (i=0; i<= GRAPH_WIDTH_PIX;++i){
            if (-32767 == fastGRA[i]){
                int16_t alt = getBaroAlt(fastGRP[i]*10L, fastPRef, fastGRT[i]*10);
                if (alt>9999) alt=9999;
                if (alt<-999) alt=-999;
                fastGRA[i]=alt;
            }
        }
    }
    if (graphDewTemp(graphInterpretationMode)){
        for (i=0; i<= GRAPH_WIDTH_PIX;++i){
            if (32767 == fastGRD[i]){
                fastGRD[i] = getDewPoint(fastGRT[i]*10,fastGRRH[i])/10;
            }
        }
    }
}

static void insertFastData(int16_t temp, int16_t pres, uint8_t rh){
    int pos = fastDataPos + 1;
    if (pos > GRAPH_WIDTH_PIX) pos = 0;
    fastGRT[pos]=temp;
    fastGRP[pos]=pres;
    fastGRRH[pos]=rh;
    fastGRD[pos]=32767;
    fastGRA[pos]=-32767;
    fastDataPos = pos;
}

int graphProcessData(int pres, int temp, int rh){
    uint32_t cur_cycles = k_cycle_get_32();
    uint32_t lim = sys_clock_hw_cycles_per_sec();
    int rv = 0;
    if ((cur_cycles - last_shift_cycles_fast) > lim) {
        pres = (pres + 5) / 10;
        temp = (temp + ((temp >= 0) ? 5 : -5)) / 10;
        if ((cur_cycles - last_shift_cycles_fast) < 10 * lim) {
            last_shift_cycles_fast += lim;
        } else {
            last_shift_cycles_fast = cur_cycles - 9 * lim;
        }
        insertFastData(temp, pres, rh);
        fastUpdated = 1;
        rv = 1;
    }
    return rv;
}

static void fixLimit(int * limit){
    int i;
    unsigned int j;
    unsigned int pow10=0;
    i = *limit;
    if (i < 0) i = -i;
    while (i>10) {pow10++;i=1+i/10;}
    //for(pow10=0;i>10;i/=10) ++pow10;
    if (i>5) i=10;
    for (j=0;j<pow10;j++) i*=10;
    *limit = i;
}

//#ifdef MINIDIGS_7X6
#define MINI_DIG_H 8
#define MINI_DIG_W 6
// size (7x6)
const uint8_t minidig_tab[19][MINI_DIG_H] = {
{ 0x70, 0x88, 0x88, 0x88, 0x88, 0x88, 0x70, 0x00}, //0
{ 0x20, 0x60, 0x20, 0x20, 0x20, 0x20, 0x70, 0x00}, //1
{ 0x70, 0x88, 0x08, 0x30, 0x40, 0x80, 0xf8, 0x00}, //2
{ 0x70, 0x88, 0x08, 0x30, 0x08, 0x88, 0x70, 0x00}, //3
{ 0x10, 0x30, 0x50, 0x90, 0xf8, 0x10, 0x38, 0x00}, //4
{ 0xf8, 0x80, 0x80, 0xf0, 0x08, 0x88, 0x70, 0x00}, //5
{ 0x70, 0x88, 0x80, 0xf0, 0x88, 0x88, 0x70, 0x00}, //6
{ 0xf8, 0x88, 0x08, 0x10, 0x10, 0x20, 0x20, 0x00}, //7
{ 0x70, 0x88, 0x88, 0x70, 0x88, 0x88, 0x70, 0x00}, //8
{ 0x70, 0x88, 0x88, 0x78, 0x08, 0x88, 0x70, 0x00}, //9
{ 0x70, 0x88, 0x88, 0x88, 0xf8, 0x88, 0x88, 0x00}, //a
{ 0xf0, 0x88, 0x88, 0xf0, 0x88, 0x88, 0xf0, 0x00}, //b
{ 0xf0, 0x88, 0x80, 0x80, 0x80, 0x88, 0xf0, 0x00}, //c
{ 0xf0, 0x88, 0x88, 0x88, 0x88, 0x88, 0xf0, 0x00}, //d
{ 0xf8, 0x80, 0x80, 0xf0, 0x80, 0x80, 0xf8, 0x00}, //e
{ 0xf8, 0x80, 0x80, 0xf0, 0x80, 0x80, 0x80, 0x00}, //f
{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, // space
{ 0x00, 0x00, 0x00, 0x70, 0x00, 0x00, 0x00, 0x00}, // minus
{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x00}, // dot
};

#if 0
static void minidig_put_line_4bit(uint8_t * ptr, int c, int line, uint32_t fc, uint32_t bc)
{
    if (c > sizeof(minidig_tab)/sizeof(minidig_tab[0][0])/MINI_DIG_H) c = 0;
    if ((line < 0) || (line >= MINI_DIG_H)) line = MINI_DIG_H - 1;
    unsigned int ch = minidig_tab[c][line];
    int mask = 0x80;
    for(int i = 0; i < ((MINI_DIG_W+1)/2); ++i){
        uint8_t byte = (ch & mask) ? fc : bc;
        mask >>= 1;
        byte <<= 4;
        byte |= (ch & mask) ? fc : bc;
        mask >>= 1;
        *ptr = byte;
        ++ptr;
    }
}
#endif

static void minidig_put_line_4bit_stream(uint8_t * ptr, int*ofs, int* cached_col, int c, int line, uint32_t fc, uint32_t bc)
{
    if (c > sizeof(minidig_tab)/sizeof(minidig_tab[0][0])/MINI_DIG_H) c = 0;
    if ((line < 0) || (line >= MINI_DIG_H)) line = MINI_DIG_H - 1;
    unsigned int ch = minidig_tab[c][line];
    int mask = 0x80;
    for(int i = 0; i < (MINI_DIG_W); ++i){
        *cached_col = ((*cached_col) << 4) + ((ch & mask) ? fc : bc);
        mask >>= 1;
        if (0 == (++(*ofs) & 1)) ptr[(*ofs)>>1] = *cached_col;
    }
}

static void intToMiniDig(uint8_t * str, int val, int dp){
    int minus = 0;
    if (val < 0) {
        minus = 1; val = -val;
    }
    for(int pos = --dp; pos >= 0; --pos){
        if ((val) || (pos == dp)){
            str[pos] = (val % 10);
            val = val / 10;
        } else if (minus) {
            str[pos] = 0x11;
            minus = 0;
        } else {
            str[pos] = 0x10;
        }
    }
    if ((val) || (minus)){
        for(int pos = 0; pos <= dp; ++pos) str[pos] = 0x11;
    }
}

#define G_GRID_COL_MASK   0x01
#define G_MID_COL_MASK    0x02
#define G_LEFT_COL_MASK   0x04
#define G_RIGHT_COL_MASK  0x08

static const uint8_t colorMatrixNormal[16] = {
    COL_3_WHITE,   COL_3_GREEN,
    COL_3_CYAN,    COL_3_GREEN,
    COL_3_RED,     COL_3_RED, 
    COL_3_RED,     COL_3_RED, 
    COL_3_BLUE,    COL_3_BLUE,    
    COL_3_BLUE,    COL_3_BLUE,    
    COL_3_BLUE,    COL_3_BLUE,    
    COL_3_BLUE,    COL_3_BLUE,    
};

static const uint8_t colorMatrixNormalDt[16] = {
    COL_3_WHITE,   COL_3_GREEN,
    COL_3_MAGENTA, COL_3_MAGENTA,
    COL_3_RED,     COL_3_RED, 
    COL_3_RED,     COL_3_RED, 
    COL_3_BLUE,    COL_3_BLUE,    
    COL_3_BLUE,    COL_3_BLUE,    
    COL_3_BLUE,    COL_3_BLUE,    
    COL_3_BLUE,    COL_3_BLUE,    
};


static const gr_colorscheme_t csNormal = {
    .graphColMatrix = &colorMatrixNormal[0],
    .textFcol = COL_3_BLACK,
    .textBcol = COL_3_WHITE,
    .barBcol = COL_3_GREEN,
    .barLcol = COL_3_RED,
};

static const gr_colorscheme_t csNormalDt = {
    .graphColMatrix = &colorMatrixNormalDt[0],
    .textFcol = COL_3_BLACK,
    .textBcol = COL_3_WHITE,
    .barBcol = COL_3_GREEN,
    .barLcol = COL_3_RED,
};

#define COLC(acol,bcol) ((acol + (bcol << 4)) << 1)

static inline void unalignedPixelPut(uint8_t* ptr, int *lineOfsPtr, int * colValPtr, int colToPut, int count){
    int lineOfs = *lineOfsPtr;
    int colVal = *colValPtr;
    for (int j=0; j< count;j++) {
        colVal = (colVal << 4) + colToPut;
        if ( 0 == (++lineOfs & 1)) ptr[lineOfs >> 1] = colVal;
    }
    *colValPtr = colVal;
    *lineOfsPtr = lineOfs;
}

static void graphDisplay(int ymin, int ymax, gr_tickdata_t ticks, gr_labeldata_t labels, const gr_colorscheme_t * cs){
    // variables
    int i,j;
    int leftLow,leftHigh,leftBase,varA,varB,varTemp,rightLow,rightHigh,rightBase;
    int middleBase;
    int leftStep,rightStep;
    unsigned int currentCol;
    uint8_t linedata[DISP_W + 2];
    int lineBase = 0;
    int colVal;
    int lineOfs;
    int curY = ymin;
    const int gsx = ((DISP_W - (GRAPH_WIDTH_PIX + GRAPH_Y_SCALE_TICK*2 + 2 + MINI_DIG_W*(labels.leftDigs + labels.rightDigs)))/2) & 0xfffe;
    const int contentHeight = 3*MINI_DIG_H + 4 + 10*GRAPH_DEC_BLOCKS + GRAPH_Y_SCALE_TICK;
    const int pad = ymax - ymin - contentHeight;
    const int istart = -MINI_DIG_H-1 - pad/2;
    const int iend = istart + contentHeight + pad;
    char leftlabel[8];
    char rightlabel[8];
    char bottomlabel[8];
    memset(&leftlabel[0],  0x10, sizeof(leftlabel)/sizeof(leftlabel[0]));
    memset(&rightlabel[0], 0x10, sizeof(rightlabel)/sizeof(rightlabel[0]));    
    memset (linedata, COLC(cs->textBcol,cs->textBcol), DISP_W + 2);
    //coords

    // Get leftLow,leftHigh,rightLow,rightHigh - lowest and highest temperature and pressure
    leftLow = dataLeft[0]; leftHigh = leftLow;
    rightLow = dataRight[0];rightHigh = rightLow;
    for (j=1; j<GRAPH_WIDTH_PIX; j++){
        leftBase=dataLeft[j]; if(leftLow>leftBase)leftLow=leftBase; if(leftHigh<leftBase)leftHigh=leftBase;
        rightBase=dataRight[j]; if(rightLow>rightBase)rightLow=rightBase; if(rightHigh<rightBase)rightHigh=rightBase;
    }
    if (!labels.middleIsBar){ // rescan limits if middle is not a bar but a graph
        for (j=0; j<GRAPH_WIDTH_PIX; j++){
            leftBase=dataMiddle[j]; if(leftLow>leftBase)leftLow=leftBase; if(leftHigh<leftBase)leftHigh=leftBase;
        }
    }
    // process limits;
    leftStep=1+(leftHigh-leftLow)/(10*(GRAPH_DEC_BLOCKS-1)); // left scale [temperature] step
    fixLimit(&leftStep);
    if (leftLow>0) {
       leftBase=(10*leftStep)*(GRAPH_DEC_BLOCKS+(leftLow-leftStep)/(10*leftStep));
    } else {
        leftBase=(10*leftStep)*(GRAPH_DEC_BLOCKS-1+(leftLow-leftStep)/(10*leftStep));             //top line value padded to bottom
    }
    rightStep=1+(rightHigh-rightLow)/(10*(GRAPH_DEC_BLOCKS-1)); // right scale [pressure] step
    fixLimit(&rightStep);

    if (rightHigh>0) {
        rightBase=(10*rightStep)*(1+(rightHigh-rightStep)/(10*rightStep));
    } else {
        rightBase=(10*rightStep)*((rightHigh-rightStep)/(10*rightStep));             //top line value padded to top
    }

    leftLow=leftBase;rightLow=rightBase;
    // Draw graph and side scales
    memset (linedata, COLC(cs->textFcol,cs->textFcol), DISP_W/2);
    lcd_spi_xfer_line_async(&linedata[0], curY++, 4);
    lcd_wait_async();

    memset (linedata, COLC(cs->textBcol,cs->textBcol), DISP_W + 2);
    for (i = istart+1; i < iend -1; ++i){
        // prepare labels
        lineOfs = gsx-2;
        colVal = COLC(cs->textBcol, cs->textBcol);
        int sideLabelYraw = i + (MINI_DIG_H / 2);
        int sideLabelIndex = sideLabelYraw / 10;
        int sideLabelYphase = sideLabelYraw % 10;
        if ((sideLabelYraw >= 0) && (0 == sideLabelYphase)){
            if (sideLabelYraw <= (10*GRAPH_DEC_BLOCKS)) {
                intToMiniDig(&leftlabel[0], (leftBase/10 - sideLabelIndex*leftStep)*labels.leftRatio, labels.leftDigs);
                intToMiniDig(&rightlabel[0], (rightBase/10 - sideLabelIndex*rightStep)*labels.rightRatio, labels.rightDigs);
            } else {
                memset(&leftlabel[0],  0x10, sizeof(leftlabel)/sizeof(leftlabel[0]));
                memset(&rightlabel[0], 0x10, sizeof(rightlabel)/sizeof(rightlabel[0]));
            }
        }
        {
            for (j = 0; j < labels.leftDigs; ++j){
                minidig_put_line_4bit_stream(&linedata[lineBase], &lineOfs, &colVal, leftlabel[j], sideLabelYphase, (cs->textFcol)<<1, (cs->textBcol)<<1);
            }
        }
        if (-1 > i){
            unalignedPixelPut(&linedata[lineBase], &lineOfs, &colVal, cs->textBcol << 1, (GRAPH_Y_SCALE_TICK*2 + 2 + GRAPH_WIDTH_PIX));
        } else if (-1 == i) {
            unalignedPixelPut(&linedata[lineBase], &lineOfs, &colVal, cs->textBcol << 1, GRAPH_Y_SCALE_TICK);
            unalignedPixelPut(&linedata[lineBase], &lineOfs, &colVal, cs->textFcol << 1, (2 + GRAPH_WIDTH_PIX));
            unalignedPixelPut(&linedata[lineBase], &lineOfs, &colVal, cs->textBcol << 1, GRAPH_Y_SCALE_TICK);
        } else if ((i>=0) && (i<(10*GRAPH_DEC_BLOCKS))){                                             //graph line cycle
            int i10=i%10;
            unalignedPixelPut(&linedata[lineBase], &lineOfs, &colVal, (((i10) ? cs->textBcol : cs->textFcol) << 1), GRAPH_Y_SCALE_TICK);
            unalignedPixelPut(&linedata[lineBase], &lineOfs, &colVal, cs->textFcol << 1, 1);
            leftHigh=dataLeft[0];
            middleBase=dataMiddle[0];
            rightHigh=dataRight[0];
            int drawFlag = 0;
            int phase = ticks.gridPhase;
            for (j=0;j<(GRAPH_WIDTH_PIX);j++){
                currentCol= 0;
                if ((i10)==0) { if (((j+ticks.gridPhase)&1)==0)currentCol |= G_GRID_COL_MASK;}
                else{
                     if((i&1)==0){
                         if (0 == phase)currentCol |= G_GRID_COL_MASK;
                         phase++; 
                         if (phase==ticks.gridDist)phase = 0;
                     }
                }
                varA=dataLeft[j];varB=leftHigh;leftHigh=varA; if ((varB>=leftLow)&&(varB<leftLow+leftStep)) drawFlag=1; else drawFlag=0;
                if (varB<varA){if((drawFlag)&&(varA<leftLow+leftStep))drawFlag=0; varTemp=varB; varB=varA;varA=varTemp;}else if((drawFlag)&&(varA>=leftLow))drawFlag=0; 
                if((varB>=leftLow)&&(varA<leftLow+leftStep)&&(drawFlag==0)) currentCol |= G_LEFT_COL_MASK;
            
                varA=dataRight[j];varB=rightHigh;rightHigh=varA;if ((varB>=rightLow)&&(varB<rightLow+rightStep)) drawFlag=1; else drawFlag=0;
                if (varB<varA){if((drawFlag)&&(varA<rightLow+rightStep))drawFlag=0; varTemp=varB; varB=varA;varA=varTemp;}else if((drawFlag)&&(varA>=rightLow))drawFlag=0; 
                if((varB>=rightLow)&&(varA<rightLow+rightStep)&&(drawFlag==0)) currentCol |= G_RIGHT_COL_MASK;
                
                
                if(labels.middleIsBar){
                     if(dataMiddle[j]>=((10*GRAPH_DEC_BLOCKS)-i) ) currentCol |= G_MID_COL_MASK;
                }else{
                    varA=dataMiddle[j];varB=middleBase;middleBase=varA; if ((varB>=leftLow)&&(varB<leftLow+leftStep)) drawFlag=1; else drawFlag=0;
                    if (varB<varA){if((drawFlag)&&(varA<leftLow+leftStep))drawFlag=0; varTemp=varB; varB=varA;varA=varTemp;}else if((drawFlag)&&(varA>=leftLow))drawFlag=0; 
                    if((varB>=leftLow)&&(varA<leftLow+leftStep)&&(drawFlag==0)) currentCol |= G_MID_COL_MASK;
                }
                colVal = (colVal << 4) + ((cs->graphColMatrix[currentCol & 0xf]) << 1);
                if ( 0 == (++lineOfs & 1)) linedata[lineBase + (lineOfs >> 1)] = colVal;
            }// end j
            unalignedPixelPut(&linedata[lineBase], &lineOfs, &colVal, cs->textFcol << 1, 1);
            unalignedPixelPut(&linedata[lineBase], &lineOfs, &colVal, (((i10) ? cs->textBcol : cs->textFcol) << 1), GRAPH_Y_SCALE_TICK);
            leftLow-=leftStep; // update limits
            rightLow-=rightStep;
        }// end graph
        else if (i > (10*GRAPH_DEC_BLOCKS)){
            int yForLabel = i - (10*GRAPH_DEC_BLOCKS + 1 + GRAPH_Y_SCALE_TICK) - 2;
            int digsInLabel =(labels.refHour>=0) ? 2 : 4;
            if ((yForLabel >= 0) && (yForLabel < MINI_DIG_H)) {
                memset(&bottomlabel[0],  0x10, sizeof(bottomlabel));
                int phaseDelta = (digsInLabel*MINI_DIG_W) - 2;
                int finalPhase = ticks.labelPhase - GRAPH_Y_SCALE_TICK - 1 + phaseDelta; //phase for draw
                int labelVal = (labels.refHour >=0) ? labels.refHour : -((GRAPH_WIDTH_PIX-1) / ticks.labelPeriod)*ticks.labelPeriod;
                if (finalPhase >= ticks.labelDist){
                    finalPhase -= ticks.labelDist;
                    if (labels.refHour >=0) {
                        labelVal = (labelVal + (ticks.labelPeriod / 60)) % 24;
                    } else {
                        labelVal += ticks.labelDist;
                    }
                }
                j = 0;
                while ( j < (GRAPH_Y_SCALE_TICK*2 + 2 + GRAPH_WIDTH_PIX)){
                    ++finalPhase;
                    if ((finalPhase == ticks.labelDist) && (j < (GRAPH_Y_SCALE_TICK + 1 + GRAPH_WIDTH_PIX))){
                        finalPhase = digsInLabel*MINI_DIG_W;
                        if ((j + phaseDelta) < (GRAPH_WIDTH_PIX + GRAPH_Y_SCALE_TICK)){
                            intToMiniDig(&bottomlabel[0], labelVal, digsInLabel);
                        } else {
                            memset(&bottomlabel[0], 0x10,sizeof(bottomlabel));
                        }
                        if (labels.refHour >=0) {
                            labelVal = (labelVal + (ticks.labelPeriod / 60)) % 24;
                        } else {
                            labelVal += ticks.labelDist;
                        }
                        for (int pd = 0; pd < digsInLabel; ++pd){
                            minidig_put_line_4bit_stream(&linedata[lineBase], &lineOfs, &colVal, bottomlabel[pd], yForLabel, (cs->textFcol)<<1, (cs->textBcol)<<1);
                        }
                        j += finalPhase;
                    }
                    colVal = (colVal << 4) + (cs->textBcol << 1);
                    if ( 0 == (++lineOfs & 1)) linedata[lineBase + (lineOfs >> 1)] = colVal; 
                    ++j;                       
                }
            } else {
                int finalPhase = 1;
                int finalPeriod = DISP_W; // period
                for (j = 0; j < (GRAPH_Y_SCALE_TICK*2 + 2 + GRAPH_WIDTH_PIX); ++j){
                    if ((1+GRAPH_Y_SCALE_TICK) == j) {
                        if (i == (10*GRAPH_DEC_BLOCKS+1)){
                            finalPhase = ticks.tickPhase;
                            finalPeriod = ticks.tickDist;
                        } else if (i <= (10*GRAPH_DEC_BLOCKS+1+GRAPH_X_SCALE_TICK)){
                            finalPhase = ticks.labelPhase;
                            finalPeriod = ticks.labelDist;                        
                        }
                    } else if (((2 + GRAPH_Y_SCALE_TICK + GRAPH_WIDTH_PIX) == j) ){
                        finalPhase = 1;
                        finalPeriod = DISP_W;
                    }
                    colVal = (colVal << 4) + ((finalPhase) ? (cs->textBcol << 1) : (cs->textFcol << 1));
                    if (++finalPhase == finalPeriod) finalPhase = 0;
                    if ( 0 == (++lineOfs & 1)) linedata[lineBase + (lineOfs >> 1)] = colVal;
                }
            }
        } else  { // ((10*GRAPH_DEC_BLOCKS) == i)
            unalignedPixelPut(&linedata[lineBase], &lineOfs, &colVal, cs->textFcol << 1, (2*GRAPH_Y_SCALE_TICK+2+GRAPH_WIDTH_PIX));
        }
        //sideLabelYphase = (i + (MINI_DIG_H / 2)) % 10;        
        for (j = 0; j < labels.rightDigs; ++j){
            minidig_put_line_4bit_stream(&linedata[lineBase], &lineOfs, &colVal, rightlabel[j], sideLabelYphase, (cs->textFcol)<<1, (cs->textBcol)<<1);
        }
        //Pass 2 : bottom
        if (labels.scrollSize) {
            int yForLabel = i - (10*GRAPH_DEC_BLOCKS + 1 + GRAPH_Y_SCALE_TICK) - 2 - MINI_DIG_H;
            if ((yForLabel >= 0) && (yForLabel < MINI_DIG_H)){
                int midWidth = (GRAPH_WIDTH_PIX + GRAPH_Y_SCALE_TICK*2 + 2);
                if (midWidth < DISP_W){
                    lineOfs = 1;
                    midWidth = DISP_W - 10*MINI_DIG_W - 4;
                } else {
                    lineOfs = (((DISP_W - (midWidth + MINI_DIG_W*10))/2) & 0xfffe) - 2;
                }

                colVal = COLC(cs->textBcol, cs->textBcol);
                intToMiniDig(&bottomlabel[0], labels.leftDay, 2);
                intToMiniDig(&bottomlabel[3], labels.leftMonth, 2);
                bottomlabel[2] = 0x12;
                for (int pd = 0; pd < 5; ++pd){
                    minidig_put_line_4bit_stream(&linedata[lineBase], &lineOfs, &colVal, bottomlabel[pd], yForLabel, (cs->textFcol)<<1, (cs->textBcol)<<1);
                }

                const int actScrollLength = midWidth - 2*GRAPH_SCROLL_LIM - 1;
                const int total = labels.scrollBefore + labels.scrollAfter;
                const int scrollBarLengh = (actScrollLength*(labels.scrollSize - total))/labels.scrollSize;
                const int pixFree = actScrollLength - scrollBarLengh;
                const int pixBefore = (labels.scrollAfter) ? ((pixFree - 1) * labels.scrollBefore + (total - pixFree)+1)/total : pixFree;
                const int pixAfter = pixFree - pixBefore;

                const int fcol = ((0 == yForLabel) || (MINI_DIG_H - 1 == yForLabel)) ? cs->textBcol : cs->barBcol;
                unalignedPixelPut(&linedata[lineBase], &lineOfs, &colVal, cs->barLcol << 1, GRAPH_SCROLL_LIM);
                unalignedPixelPut(&linedata[lineBase], &lineOfs, &colVal, fcol << 1, pixBefore);
                unalignedPixelPut(&linedata[lineBase], &lineOfs, &colVal, cs->textFcol << 1, scrollBarLengh);
                unalignedPixelPut(&linedata[lineBase], &lineOfs, &colVal, fcol << 1, pixAfter);
                unalignedPixelPut(&linedata[lineBase], &lineOfs, &colVal, cs->barLcol << 1, GRAPH_SCROLL_LIM);

                unalignedPixelPut(&linedata[lineBase], &lineOfs, &colVal, cs->textBcol << 1, 1);

                intToMiniDig(&bottomlabel[0], labels.rightDay, 2);
                intToMiniDig(&bottomlabel[3], labels.rightMonth, 2);
                bottomlabel[2] = 0x12;
                for (int pd = 0; pd < 5; ++pd){
                    minidig_put_line_4bit_stream(&linedata[lineBase], &lineOfs, &colVal, bottomlabel[pd], yForLabel, (cs->textFcol)<<1, (cs->textBcol)<<1);
                }
            } else if (yForLabel == MINI_DIG_H){
                memset (&linedata[lineBase], COLC(cs->textBcol,cs->textBcol), DISP_W / 2);
            }
        }
        lcd_wait_async();
        lcd_spi_xfer_line_async(&linedata[lineBase], curY++, 4);
        lineBase = (lineBase) ? 0 : (DISP_W/2);
    }
    lcd_wait_async();
    memset (&linedata[lineBase], COLC(cs->textFcol, cs->textFcol), DISP_W/2);
    lcd_spi_xfer_line_async(&linedata[lineBase], curY++, 4);
    lcd_wait_async();    
}

void graphCalculateTicks(gr_tickdata_t * ticks, int startTime, int minPerPixel){
  // Set up time scale data
  int gridDist; // geometrical grid lines distance in pixels
  int tickDist; // geometrical minor tick lines distance in pixels
  int labelDist; // geometrical Label and large tick distance in pixels
  int varTemp;
  // Load data
  // labels are hourly
  // LaberDist*minPerPixel must be divideable by 60 
    gridDist=10;
    tickDist=5;
    labelDist=30;
  switch(minPerPixel){
    case 2:break;
    case 4: gridDist=15;tickDist=5;labelDist=15;break;
    case 6: gridDist=10;tickDist=5;labelDist=20;break;
    case 10: gridDist=6;tickDist=6;labelDist=(MINI_DIG_W<6) ? 12:18 ; break;
    case 20: gridDist=6;tickDist=6;labelDist=(MINI_DIG_W<6) ? 12:18 ; break;
    case 30: gridDist=6;tickDist=6;labelDist=(MINI_DIG_W<6) ? 12:24 ; break;
    case 0:  gridDist=10;tickDist=10;labelDist=(MINI_DIG_W<6) ? 20:30; break; // fast mode
  }
 
  // calculate parameters
  if (minPerPixel){
    ticks->gridPeriod=gridDist*minPerPixel;
    ticks->tickPeriod=tickDist*minPerPixel;
    ticks->labelPeriod=labelDist*minPerPixel;
    varTemp=startTime/minPerPixel;
    ticks->gridPhase=varTemp%gridDist;
    ticks->tickPhase=varTemp%tickDist;
    ticks->labelPhase=varTemp%labelDist;
  }else{
    ticks->gridPeriod=gridDist;
    ticks->tickPeriod=tickDist;
    ticks->labelPeriod=labelDist;
    ticks->gridPhase=(gridDist-1)-((GRAPH_WIDTH_PIX-2)%gridDist);
    ticks->tickPhase=(tickDist-1)-((GRAPH_WIDTH_PIX-2)%tickDist);
    ticks->labelPhase=(labelDist-1)-((GRAPH_WIDTH_PIX-2)%labelDist);
  }
  ticks->gridDist=gridDist;
  ticks->tickDist=tickDist;
  ticks->labelDist=labelDist;
}

static int graphInterpretationMode = 0;

static void lablesResetToDefault(gr_labeldata_t * labels, int graphInterpretationMode){
    memset(labels, 0, sizeof(*labels));
    labels->leftDigs=3;
    labels->rightDigs=4;
    labels->middleIsBar = graphDewTemp(graphInterpretationMode) ? 0 : 1;
    labels->leftRatio=1;
    labels->rightRatio = graphPressAsAlt(graphInterpretationMode)? 10 : 1;
    labels->refHour=-1;
}

bool drawGraphFast(bool force){
    force |= fastUpdated;
    if (force){
        gr_tickdata_t ticks;
        gr_labeldata_t labels;

        lablesResetToDefault(&labels, graphInterpretationMode);    

        fixFastData(graphInterpretationMode, getGlobalReferencePressure());
        int pos = fastDataPos;
        const gr_colorscheme_t * cs = &csNormal;
        if (graphDewTemp(graphInterpretationMode)){
            cs = &csNormalDt;
        }        
        for (int j= GRAPH_WIDTH_PIX-1; j >= 0; --j){
            dataLeft[j]=fastGRT[pos];   
            if (graphPressAsAlt(graphInterpretationMode)){
                dataRight[j]=fastGRA[pos];
            }else{
                dataRight[j]=fastGRP[pos];
            }
            if (graphDewTemp(graphInterpretationMode)){
                dataMiddle[j]=fastGRD[pos];
            }else{
                dataMiddle[j]=fastGRRH[pos];
            }
           --pos;
           if (pos < 0) pos = GRAPH_WIDTH_PIX;
        }
        graphCalculateTicks(&ticks, 0, 0);
        graphDisplay(MIN_LINE_DO, MAX_LINE_DO, ticks, labels, cs);
        fastUpdated = 0;
    }
    return force;
}

static const int mppVariants[] = {2, 6, 10, 30};

static struct {
    int mppIndex;
    int backShift;
    bool updated;
    bool open;
} histSettings;

static int graphGetCompressRatio(){    //Data compression ratio for graph
    int mpp = mppVariants[histSettings.mppIndex];
    return mpp / LOG_MINUTE_INTERVAL;
}

static int getGraphMinsPerPixel(){
    return mppVariants[histSettings.mppIndex];
}

static int graphGetMaxLeft(){
    return (getLogSize() / graphGetCompressRatio() - GRAPH_WIDTH_PIX);
}

bool graphFastKeys(int key, int count){
    bool hooked = false;
    if ((0 == count) && (MSM_KEY_UP == key)){
        graphInterpretationMode = (1 + graphInterpretationMode) & 3;
        hooked = true;
    } else if ((0 == count) && (MSM_KEY_DOWN == key)){
        graphInterpretationMode = (-1 + graphInterpretationMode) & 3;
        hooked = true;
    }
    return hooked;
}

bool graphHistKeys(int key, int count){
    bool hooked = false;
    if (histSettings.open && (MSM_KEY_ESC == key) && (0 == count)){
        hooked = true;
        histSettings.open = false;
        histSettings.updated = true;
        histSettings.backShift = 0;
    } else if (MSM_KEY_UP == key) {
        if (histSettings.open) {
            histSettings.backShift -= (count + 1);
            if (histSettings.backShift < 0) histSettings.backShift = 0;
            hooked = true;
            histSettings.updated = true;
        } else {
            if (0 == count){
                if (++histSettings.mppIndex >= sizeof(mppVariants)/sizeof(mppVariants[0])) histSettings.mppIndex = 0;
                hooked = true;
                histSettings.updated = true;
            }
        }
    } else if (MSM_KEY_DOWN == key) {
        if (histSettings.open) {
            histSettings.backShift += (count + 1);
            if (histSettings.backShift > graphGetMaxLeft()) histSettings.backShift = graphGetMaxLeft();
            hooked = true;
            histSettings.updated = true;
        } else {
            if (0 == count){
                histSettings.open = true;
                hooked = true;
                histSettings.updated = true;
                histSettings.backShift = 0;
            }
        }
    }
    return hooked;
}
static int lastLogPos;
static int lastLogT;
static int lastLogP;
static int lastLogRH;

bool drawGraphHist(bool force){
    force |= histSettings.updated;

    int logPos;
    uint32_t logSecs;
    getLogPosTime(&logPos, &logSecs);
    {// if we really need to update?
        int curT = getLogT(logPos);
        int curP = getLogP(logPos);
        int curRH = getLogRH(logPos);
        if (logPos != lastLogPos) {
            force = true;
            lastLogPos = logPos;
        }
        int dT = curT > lastLogT ? curT - lastLogT : lastLogT - curT;
        int dP = curP > lastLogP ? curP - lastLogP : lastLogP - curP;
        int dRH = curRH > lastLogRH ? curRH - lastLogRH : lastLogRH - curRH;

        if ((dT > 1) || (dP > 1) || (dRH > 1)) {
            force = true;
        }
        if (force){
            lastLogP = curP;
            lastLogT = curT;
            lastLogRH = curRH;
        }
    }
    if (force){
        gr_tickdata_t ticks;
        gr_labeldata_t labels;
    
        lablesResetToDefault(&labels, graphInterpretationMode);

        logSecs += rtc_tz_delta(logSecs);
        logSecs = (60 * LOG_MINUTE_INTERVAL)*((logSecs + 30*LOG_MINUTE_INTERVAL)/(60*LOG_MINUTE_INTERVAL));
        uint32_t rightMins = logSecs/60;

        int compRatio = getGraphMinsPerPixel() / LOG_MINUTE_INTERVAL;
        int compPhase = 1 + ((rightMins) % getGraphMinsPerPixel()) / LOG_MINUTE_INTERVAL;
        int backShift = histSettings.backShift;

        labels.scrollSize = (histSettings.open) ? getLogSize()/ compRatio : 0;
        labels.scrollAfter = backShift;
        labels.scrollBefore = graphGetMaxLeft() - backShift;

        int curPos = logPos;
        if (backShift){
            curPos -= compPhase;
            curPos -= (backShift-1) * compRatio;
            compPhase = compRatio;
            if (curPos < 0) curPos += getLogSize();
            rightMins -= backShift * getGraphMinsPerPixel();
            rightMins = (rightMins / (getGraphMinsPerPixel())) * getGraphMinsPerPixel();
        }
        uint32_t pRef = getGlobalReferencePressure();
        for (int i = (GRAPH_WIDTH_PIX-1); i >=0; --i){
            int avgT = 0;
            int avgP = 0;
            int avgRH = 0;
            for (int j = 0; j < compPhase; ++j){
                avgT += getLogT(curPos);
                avgP += getLogP(curPos);
                avgRH += getLogRH(curPos);
                curPos = (curPos) ? curPos - 1 : getLogSize() - 1;
            }
            int finT = avgT / compPhase;
            int finP = avgP / compPhase;
            int finRH = avgRH / compPhase;
            dataLeft[i] = finT;
            if (graphDewTemp(graphInterpretationMode)){
                dataMiddle[i] = getDewPoint(finT*10,finRH)/10;
            } else {
                dataMiddle[i] = (10 == GRAPH_DEC_BLOCKS) ? finRH : (finRH * GRAPH_DEC_BLOCKS)/10;
            }
            if (graphPressAsAlt(graphInterpretationMode)){
                int16_t alt = getBaroAlt(finP*10L, pRef, finT*10);
                if (alt>9999) alt=9999;
                if (alt<-999) alt=-999;
                dataRight[i] = alt;
            } else {
                dataRight[i] = finP;
            }
            compPhase = compRatio;
        }
        uint32_t leftMins = rightMins - getGraphMinsPerPixel()*(GRAPH_WIDTH_PIX-1);
        int startMins = leftMins % (24*60);
//        LOG_INF("%d:%d %d:%d", startMins/60, startMins %60, (rightMins / 60) % 24, rightMins%60);
        rtc_days_to_date(leftMins/24/60, &labels.leftDay, &labels.leftMonth, NULL);
        rtc_days_to_date(rightMins/24/60, &labels.rightDay, &labels.rightMonth, NULL);
        graphCalculateTicks(&ticks, startMins, getGraphMinsPerPixel());
        {
            int labelPeriodHour =  ticks.labelPeriod/60;
            int firstHour = labelPeriodHour*(startMins/ticks.labelPeriod);
            firstHour = (labelPeriodHour + firstHour) % 24;
            // LOG_INF("First Hour = %d, phase = %d",firstHour, ticks.labelPhase);
            labels.refHour = firstHour;
        }
        const gr_colorscheme_t * cs = &csNormal;
        if (graphDewTemp(graphInterpretationMode)){
            cs = &csNormalDt;
        }    
        graphDisplay(MIN_LINE_DO, MAX_LINE_DO, ticks, labels, cs);
        fastUpdated = 0;
    }
    histSettings.updated = false;
    return force;
}

#endif