#include "dispservice.h"
#include <string.h>
#include <math.h>

#include <bluetooth/conn.h>
#include "lpm013m126.h"
#include "slowsensors.h"
#include "fonts.h"
#include "bt.h"
#include "menu.h"
#include "msm/msm.h"
#include "rtc.h"
#include "ui-config.h"
#include "calculs/meteomath.h"
#include "calculs/daynightimpl.h"
#include "charger.h"

#define LOG_LEVEL 4
#include <logging/log.h>
LOG_MODULE_REGISTER(disps);

#if UI_PRESENT

#include "../fonts/font4clock.c"

#ifndef CONFIG_NEWLIB_LIBC
#define lroundf(a) (long int)(a + 0.5f)
#define hypotf(a, b)  ((a > 0 ? a : -a) + (b > 0 ? b : -b))
#endif

static uint32_t g_pRef = 76000;

uint32_t getGlobalReferencePressure(){
    return g_pRef;
}

void setGlobalReferencePressure(uint32_t newPref){
    if (newPref < PREF_MIN_LIM) newPref = PREF_MIN_LIM;
    if (newPref > PREF_MAX_LIM) newPref = PREF_MAX_LIM;
    g_pRef = newPref;
}

static void brline(uint8_t * linedata, int y, uint8_t col, int size, int lineCount = 1){
    memset(linedata, col, size);
    for (int i = 0; i < lineCount; ++i){
        lcd_spi_xfer_line_async(&linedata[0], y+i, 4);
        lcd_wait_async();
    }
}

bool displayInfoBar(bool force){
static int last_temp = 0;
static int last_pres = 0;
static int last_rh = 0;
static uint16_t last_bt = 0x2020;
static uint32_t last_batt = 0x2020;

    char sbuf[CHARS_PER_DISP];         // string print buffer
    uint8_t cbuf[CHARS_PER_DISP];      // color print buffer
    uint8_t linedata[LINE_BUF_L];        // line buffer
    int temp = (slowsensorsGetTemperature(0) + 5) / 10;
    int pres = (slowsensorsGetPressure(FLAG_SENSOR_PRESSURE_MMHG) + 5)/10;
    int rh = slowsensorsGetHumidity(0);
    uint16_t bt = 0;
    if (bt_adv_status_get()) {
        bt |= 0x82;
    } else {
        bt |= 0x20;
    }
    if (bt_conn_count_get()){
        bt |= (bt_max_sec_get() >= BT_SECURITY_L2) ? 0x8300 : 0x8400;
    } else {
        bt |= 0x2000;
    }
    uint32_t batt = 0x8a88;
    /*batt_val_mock = (1 + batt_val_mock);
    int batt_val = batt_val_mock >> 6;
    switch(batt_val){
        case 0:
            batt = 0x8986;
            break;
        case 1:
            batt = 0x8987;
            break;
        case 2:
            batt = 0x8988;
            break;
        default:
            break;
    }*/
    {
        int bp = getBattPercentage();
        if (bp < 20){
            batt = 0x8986;
        } else if (bp < 50){
            batt = 0x8987;
        } else if (bp < 88){
            batt = 0x8988;
        }
        if (charger_is_charging()){
            batt |= ((uint32_t)COL2(COL_3_GREEN , COL_INFOBAR_BACK)) << 16;
        } else if (charger_power_good()){
            batt |= ((uint32_t)COL2(COL_3_BLUE , COL_INFOBAR_BACK)) << 16;
        } else {
            if (bp < 9){
                batt |= ((uint32_t)COL2(COL_3_RED , COL_INFOBAR_BACK)) << 16;
            } else {
                batt |= ((uint32_t)COL2(COL_INFOBAR_FRONT, COL_INFOBAR_BACK)) << 16;
            }
        }
    }

    if ((last_temp != temp) || (last_pres != pres) || (last_rh != rh) || (last_bt != bt) || (last_batt != batt)) force = true;
    if (force){
        int ipos = 0;
#if (USE_LARGER_FONT)
        brline(linedata, INFO_BAR_Y, COL2(COL_STATUSBAR_BACK, COL_STATUSBAR_BACK), sizeof(linedata));
#endif        

        memset(cbuf, COL2(COL_INFOBAR_FRONT, COL_INFOBAR_BACK), sizeof(cbuf));
        memset(sbuf,' ',sizeof(sbuf));
        int no_pres = 0;
        int total_pad = CCHARS_PER_DISP -2 -5 -3 -5 -2;
        if (total_pad < 0){
            total_pad += 5;
            no_pres = 1;
        }
        int pad_1 = total_pad / (4 - no_pres);
        int pad_2 = pad_1;
        int pad_3 = pad_2;
        int pad_4 = pad_3;
        int pad_rest = total_pad - (pad_1 * (4 - no_pres));
        if (pad_rest > 1) {
            pad_3++;
            pad_rest--;
        }
        int x = (DISP_W - (CCHAR_W)*(CCHARS_PER_DISP) + (CCHAR_W*pad_rest)) >> 1;
        cbuf[ipos] = (batt >> 16) & 0xff;
        sbuf[ipos++] = batt & 0xff;
        cbuf[ipos] = (batt >> 16) & 0xff;
        sbuf[ipos++] = (batt >> 8) & 0xff;
        ipos += pad_1;

        int cw = 5;
        itoSPadded(temp, &sbuf[ipos], cw, cw - 1);
        memset(&cbuf[ipos], COL2(COL_INFOBAR_TEMP, COL_INFOBAR_BACK), cw);
        ipos += cw;
        sbuf[ipos] = ' ';
        ipos += pad_2;

        cw = 3;
        itoSPadded(rh, &sbuf[ipos], cw, 0);
        memset(&cbuf[ipos], COL2(COL_INFOBAR_RH, COL_INFOBAR_BACK), cw);
        ipos += cw;
        sbuf[ipos] = ' ';
        ipos += pad_3;

        if (!no_pres){
            cw = 5;
            itoSPadded(pres, &sbuf[ipos], cw, cw - 1);
            memset(&cbuf[ipos], COL2(COL_INFOBAR_PRES, COL_INFOBAR_BACK), cw);
            ipos += cw;
            sbuf[ipos] = ' ';
            ipos += pad_4;
        }

        sbuf[ipos++] = bt & 0xff;
        sbuf[ipos++] = bt >> 8;  
        memset(linedata, COL2(COL_INFOBAR_BACK, COL_INFOBAR_BACK),sizeof(linedata));

        int py = (USE_LARGER_FONT) ? 1 : 0;
        PRINT_STR_COL_EXTBUF_CL(linedata, x, INFO_BAR_Y + py, sbuf, cbuf, CCHARS_PER_DISP);
#if (USE_LARGER_FONT)
        brline(&linedata[0], INFO_BAR_Y + py + CCHAR_H, COL2(COL_STATUSBAR_BACK, COL_STATUSBAR_BACK), sizeof(linedata));
#endif        

        last_temp = temp;
        last_pres = pres;
        last_rh = rh;
        last_bt = bt;
        last_batt = batt;
    }
    return force;
}

static int put_4_bar(uint8_t * linedata, uint8_t y, uint32_t mask){
    if ((1 << y) & mask){
        linedata[0] = COL2(COL_STATUSBAR_BACK, COL_STATUSBAR_BACK);       
        linedata[1] = COL2(COL_STATUSBAR_FRONT, COL_STATUSBAR_FRONT);
    } else {
        linedata[0] = COL2(COL_STATUSBAR_BACK, COL_STATUSBAR_BACK);
        linedata[1] = COL2(COL_STATUSBAR_BACK, COL_STATUSBAR_BACK);
    }
    return 4;
}

bool displayStatusBar(bool force){
static uint32_t  last_time_secs = 0;
    char sbuf[CHARS_PER_DISP];         // string print buffer
    uint8_t cbuf[CHARS_PER_DISP];      // color print buffer
    uint8_t linedata[LINE_BUF_L];        // line buffer
    uint32_t time_secs = rtc_time_secs_get_tz();
    if (last_time_secs != time_secs) force = true;
    if (force){
        uint8_t hour, min, sec, day, month, year;
        rtc_get_date_time_from_secs(time_secs, &day, &month, &year, &hour, &min, &sec);
        memset(cbuf, COL2(COL_STATUSBAR_FRONT, COL_STATUSBAR_BACK), sizeof(cbuf));
#if (USE_LARGER_FONT)
        brline(&linedata[0], STATUS_BAR_Y, COL2(COL_STATUSBAR_BACK, COL_STATUSBAR_BACK), sizeof(linedata));
        itoSPadded(hour, &sbuf[0], 2, 0);
        itoSPadded(min, &sbuf[3], 2, 0xff);
        itoSPadded(sec, &sbuf[6], 2, 0xff);
        int passkey;
        if ((passkey = passkey_get()) >= 0){
            itoSPadded(passkey, &sbuf[9], 6, 0xff);
        } else {
            itoSPadded(day, &sbuf[9], 2, 0);
            itoSPadded(month, &sbuf[12], 2, 0xff);
            itoSPadded(year, &sbuf[15], 2, 0xff);
        }
        int xpad = DISP_W - 12*CCHAR_W - 4*6;
        lcd_wait_async();          
        for (int y = 0; y < CCHAR_H; ++y){
            unsigned int x = 4;
            for (int i = 0; 1; ++i){
                PUT_STR_COL_EXTBUF_CL(linedata, x, y, &sbuf[i*3], cbuf, 2);
                x += CCHAR_W << 1;
                if (i >= 2) break;
                x += put_4_bar(&linedata[x >> 1], y , 0x1830);
            }
            x += xpad;
            if (passkey >= 0){
                linedata[(x++ >> 1)] = COL2(COL_PASSKEY_BACK, COL_PASSKEY_BACK);
                linedata[(x++ >> 1)] = COL2(COL_PASSKEY_BACK, COL_PASSKEY_BACK);
                memset(&cbuf[9], COL2(COL_PASSKEY_FRONT, COL_PASSKEY_BACK), 6);
                PUT_STR_COL_EXTBUF_CL(linedata, x, y, &sbuf[9], &cbuf[9], 6);
                x += CCHAR_W * 6;
                linedata[(x++ >> 1)] = COL2(COL_PASSKEY_BACK, COL_PASSKEY_BACK);
                linedata[(x++ >> 1)] = COL2(COL_PASSKEY_BACK, COL_PASSKEY_BACK);
            } else {
                for (int i = 0; 1; ++i){
                    PUT_STR_COL_EXTBUF_CL(linedata, x, y, &sbuf[i*3 + 9], cbuf, 2);
                    x += CCHAR_W << 1;
                    if (i >= 2) break;
                    x += put_4_bar(&linedata[x >> 1], y , 0x6000);
                }
            }
            lcd_spi_xfer_line_async(&linedata[0], STATUS_BAR_Y + 1 + y, 4);
            lcd_wait_async();            
        }
        brline(&linedata[0], STATUS_BAR_Y + 1 + CCHAR_H, COL2(COL_STATUSBAR_BACK, COL_STATUSBAR_BACK), sizeof(linedata));
#else        
        memset(sbuf,' ',sizeof(sbuf));
        char * pstr = &sbuf[2];
        int w = 2;
        pstr = itoSPadded(hour, pstr, w, 0) + w;
        *pstr++ = MSM_TIME_SEPARATOR;
        pstr = itoSPadded(min, pstr, w, 0xff) + w;
        *pstr++ = MSM_TIME_SEPARATOR;
        pstr = itoSPadded(sec, pstr, w, 0xff) + w;
        *pstr++ = ' ';
        pstr = &sbuf[CHARS_PER_DISP-8-2];
        int passkey;
        if ((passkey = passkey_get()) >= 0){
            w = 6;
            pstr--;
            *pstr++ = '\202';
            *pstr++ = '\205';
            pstr = itoSPadded(passkey, pstr, w, 0xff) + w;
            memset(&cbuf[CHARS_PER_DISP-8-2], COL2(COL_PASSKEY_FRONT, COL_PASSKEY_BACK), 7);
            cbuf[CHARS_PER_DISP-8-2-1] = COL2(COL_PASSKEY_BACK, COL_PASSKEY_FRONT);
            cbuf[CHARS_PER_DISP-8-2+7] = COL2(COL_PASSKEY_BACK, COL_PASSKEY_FRONT);
            cbuf[CHARS_PER_DISP-8-2+8] = COL2(COL_PASSKEY_BACK, COL_PASSKEY_FRONT);
            *pstr++='\205';
            *pstr++ = '\203';
        } else {
            pstr = itoSPadded(day, pstr, w, 0) + w;
            *pstr++ = MSM_DATE_SEPARATOR;
            pstr = itoSPadded(month, pstr, w, 0xff) + w;
            *pstr++ = MSM_DATE_SEPARATOR;
            pstr = itoSPadded(year, pstr, w, 0xff) + w;
            *pstr++ = ' ';
        }
        print_str_col_extbuf(linedata, 0, STATUS_BAR_Y, sbuf, cbuf, CHARS_PER_DISP);
#endif        
        last_time_secs = time_secs;
    }
    return force;
}

static const uint8_t sunClockColors[DAY_NIGHT_VALS] = {
    COL_3_YELLOW,
    COL_3_RED,
    COL_3_CYAN,
    COL_3_BLUE,
    COL_3_BLUE,
};

#define DAY_NIGHT_ARROW_COLOR COL_3_GREEN

static void renderSunClock(uint8_t * linedata, uint8_t * pixmap, int y, int h, int pos){
    int i,j,val;
    uint8_t col;
    const int w = DISP_W;
    int arrowSize = (h + 3) >> 2;
    int step = arrowSize >> 1;
    for (j = 0; j < h; ++j){
        int arrowDelta =  j < (h >> 1) ? arrowSize - j : arrowSize - (h - 1 - j);
        for (i = 0;  i < w;  ++i){
            col = COL_3_BLACK;
            int arrowX = (i > pos) ? (i - pos) : (pos - i);
            if ( arrowDelta > arrowX ) {
                col = DAY_NIGHT_ARROW_COLOR;
            } else if ((j < step) || (j + step >= h)) {
                col = COL_3_BLACK;
            } else {
                if ((val = pixmap[i]) < 0xf) col = sunClockColors[val];
                //if ((DAY_NIGHT_VALS - 1 == val) && ((i+j) & 1)) col = COL_3_BLACK;
                if ((DAY_NIGHT_VALS - 1 == val) && ((j+(i<<1)) & 3)) col = COL_3_BLACK;
            }
            if (i & 1) {
                linedata[i >> 1] |= (col << 1);
            } else {
                linedata[i >> 1] = (col << 5);
            }
        }
        lcd_spi_xfer_line_async(&linedata[0], j + y, 4);
        lcd_wait_async();
    }
}

static inline void fillGap(uint8_t * buf, int min, int max, int w, int val){
    if (min >= w) min = w;
    if (min < 0) min = 0;
    if (max >= w) max = w;
    if (max < 0) max = 0;
    if (max > min) memset(&buf[min], val, max-min);
}

static void displaySunClock(uint8_t * linedata, int y, int h, uint32_t time, int mode){
    int i,p1,p2,pos;
    // linedata will keep DISP_W number of bytes of index data and DISP_W/2 bytes of libe buffer
    uint8_t * pixmap = linedata;
    float noon, currentTime;
    float shift = 0.0f;
    const int w = DISP_W;
    memset(pixmap, 0xff, w);

    if (dn_gap[DAY_NIGHT_RISE_N].end >= dn_gap[DAY_NIGHT_RISE_N].start){
        noon = (dn_gap[DAY_NIGHT_RISE_N].end + dn_gap[DAY_NIGHT_RISE_N].start)/2.0f;
    } else {
        noon = fmod24(12.0f + (dn_gap[DAY_NIGHT_RISE_N].end + dn_gap[DAY_NIGHT_RISE_N].start)/2.0f);
    }
    currentTime = ((float) (time % 86400)) / 3600.0f;
    if (sunClockCenterNoon == mode){
        shift = (noon - 12.0f);
    } else if (sunClockCenterCurrent == mode){
        shift = (currentTime - 12.0f);
    } else if (sunClockNormal == mode){
        shift = - rtc_tz_delta(time) / 3600.0f;
    }
    for(i=(DAY_NIGHT_VALS-1); i >= 0; --i){
        if (DayNightAlways == dn_state[i]){
            fillGap(pixmap, 0, w, w, i);
        } else if (DayNightNormal == dn_state[i]) {
            float start = fmod24(dn_gap[i].start - shift);
            float end = fmod24(dn_gap[i].end - shift);
            p1 = (w)*start / 24.0f;
            p2 = (w)*end / 24.0f;
            if (end > start) {
                fillGap(pixmap, p1, p2, w, i);
            } else {
                fillGap(pixmap, 0, p2, w, i);
                fillGap(pixmap, p1, w, w, i);
            }
        }
    }
    pos = (w * fmod24(currentTime - shift)) / 24.0f;
    renderSunClock(&linedata[DISP_W], &linedata[0], y, h, pos);
}

static uint8_t dclockCols[2] = {COL_DCLOCK_BACK, COL_DCLOCK_FRONT};

// This code can be used for out of order line draw, but we do not need it now
// because we droy line by line from top to bottom
#if 0
static int dclockGetOfsForLine(int dig, int line){
    int cline = 0;
    for (int ofs = clock_sym_offsets[dig]; ofs < clock_sym_offsets[dig+1]; ++ofs){
        uint8_t val = clock_sym_data[ofs];
        if (val & 0x80){
            cline += val & (0x7f);
            continue;
        } else if (val & 0x40) {
            cline +=1;
            continue;
        } else if (cline == line){
            return ofs;
        } else if (cline > line){
            return -1;
        }

    }
    return -1;
}

static void dclockDrawLineUnpacked(uint8_t *linedata, int dig, int x, int line, int wlim){
    int n = 0;
    int xpos = x;
    int lofs = clock_sym_offsets[dig+1];
    int ofs = dclockGetOfsForLine(dig, line);
    while ((ofs >= 0) && (wlim)){
        uint8_t val = clock_sym_data[ofs++];
        if ((ofs >= lofs) || (val & 0xc0)) ofs = -1;
        val &= 0x3f;
        if (val >= wlim) val = wlim;
        uint8_t pcol = dclockCols[n++ & 1];
        memset(linedata + xpos, pcol, val);
        xpos += val;
        wlim -= val;
    }
    if (wlim) memset(linedata + xpos, dclockCols[ n & 1], wlim);
}
#endif

static int16_t dclockOfs[5];
static int16_t dclockCline[5];

static void dclockDrawLineUnpackedSerial(uint8_t *linedata, int dig, int x, int line, int wlim, int index){
    if (0 == line) {
        dclockOfs[index] = clock_sym_offsets[dig];
        dclockCline[index] = 0;
    }
    int n = 0;
    int xpos = x;
    int lofs = clock_sym_offsets[dig+1];
    int ofs = dclockOfs[index];
    int cline = dclockCline[index];
    if (line == cline){
        while ((ofs >= 0) && (wlim)){
            uint8_t val = clock_sym_data[ofs++];
            if ((ofs >= lofs) || (val & 0xc0)) {
                if ((ofs >= lofs)){
                    cline = -1;
                } else if (val & 0x80){
                    cline += val & (0x7f);
                } else if (val & 0x40) {
                    cline += 1;
                }
                dclockCline[index] = cline;
                dclockOfs[index] = ofs;
                if (val & 0x80) break;
                ofs = -1;
            }
            val &= 0x3f;
            if (val >= wlim) val = wlim;
            uint8_t pcol = dclockCols[n++ & 1];
            memset(linedata + xpos, pcol, val);
            xpos += val;
            wlim -= val;
        }
    }
    if (wlim) memset(linedata + xpos, dclockCols[n & 1], wlim);
}

typedef char daystr_t[16];

static const daystr_t day_names[8]={
    "noday",
    "Monday",
    "Tuesday",
    "Wednesday",
    "Thursday",
    "Friday",
    "Saturday",
    "Sunday"
};

static void dclockDraw(int hour, int min, int dow, uint32_t time_secs_utc, int minLine = MIN_LINE_DO, int maxLine = MAX_LINE_DO){
    uint8_t linedata[LINE_BUF_L*3]; //We expose unpacked data in buffer
    int space_above = (maxLine - minLine - clock_sym_h) / 2;
    int space_below = (maxLine - minLine - clock_sym_h) - space_above;

    if ((space_below < CCHAR_H) && ((space_above << 1) > CCHAR_H)){
        space_below = CCHAR_H;
        space_above = (space_above << 1) - space_below;
    }

    /*memset (linedata,0,DISP_W / 2);
    for (int y = minLine; y < start_y; ++y){
        lcd_spi_xfer_line_async(&linedata[0], y, 4);
        lcd_wait_async();
    }*/
    {
        int y = minLine;
        const int sun_clock_pad = (CCHAR_H + 2) / 4;
        const int sun_clock_h = space_above - 2 * sun_clock_pad;
        brline(linedata, y, COL2(COL_3_BLACK, COL_3_BLACK), DISP_W/2, sun_clock_pad);
        y += sun_clock_pad;
        displaySunClock(linedata, y, sun_clock_h, time_secs_utc, sunClockNormal);
        y += sun_clock_h;
        brline(linedata, y, COL2(COL_3_BLACK, COL_3_BLACK), DISP_W/2, sun_clock_pad);
    }
    int start_y = minLine + space_above;
    for (int y = 0; y < clock_sym_h; ++y){
        int delim_w = DISP_W - (clock_sym_w << 2);
        int xpos = 0;
        int ofs = (DISP_W/2) * (y & 1);
        /*dclockDrawLineUnpacked(&linedata[ofs], hour / 10, xpos, y, clock_sym_w);
        xpos += clock_sym_w;
        dclockDrawLineUnpacked(&linedata[ofs], hour % 10, xpos, y, clock_sym_w);
        xpos += clock_sym_w;
        dclockDrawLineUnpacked(&linedata[ofs], 10, xpos, y, delim_w);
        xpos += delim_w;
        dclockDrawLineUnpacked(&linedata[ofs], min / 10, xpos, y, clock_sym_w);
        xpos += clock_sym_w;
        dclockDrawLineUnpacked(&linedata[ofs], min % 10, xpos, y, clock_sym_w);*/
        dclockDrawLineUnpackedSerial(&linedata[ofs], hour / 10, xpos, y, clock_sym_w, 0);
        xpos += clock_sym_w;
        dclockDrawLineUnpackedSerial(&linedata[ofs], hour % 10, xpos, y, clock_sym_w, 1);
        xpos += clock_sym_w;
        dclockDrawLineUnpackedSerial(&linedata[ofs], 10, xpos, y, delim_w, 2);
        xpos += delim_w;
        dclockDrawLineUnpackedSerial(&linedata[ofs], min / 10, xpos, y, clock_sym_w, 3);
        xpos += clock_sym_w;
        dclockDrawLineUnpackedSerial(&linedata[ofs], min % 10, xpos, y, clock_sym_w, 4);
        for (int x = 0; x < (DISP_W >> 1); ++x){
            linedata[ofs + x] = ((linedata[ofs + (x << 1)] << 4) + linedata[ofs + (1 + (x << 1))]) << 1;
        }
        lcd_wait_async();
        lcd_spi_xfer_line(&linedata[ofs], start_y + y, 4);
    }
    lcd_wait_async();
    int cur_y = start_y + clock_sym_h;
    if ((dow) && (space_below >= CCHAR_H)){
        uint8_t cbuf[CHARS_PER_DISP];
        memset (linedata, 0, DISP_W / 2);
        int lim_y = cur_y + ((space_below - CCHAR_H) >> 1);
        for (int y = cur_y; y < lim_y; ++y){
            lcd_spi_xfer_line_async(&linedata[0], y, 4);
            lcd_wait_async();
        }          
        cur_y = lim_y;
        memset (cbuf, COL2(COL_DCLOCK_FRONT, COL_DCLOCK_BACK), CHARS_PER_DISP);
        int length = strlen(day_names[dow]);
        uint32_t x = (DISP_W - (CCHAR_W)*length) >> 1;
        const char * str = (const char*) &day_names[dow][0];
        PRINT_STR_COL_EXTBUF_CL(linedata, x, cur_y, str, cbuf, length);
        cur_y += CCHAR_H;
    }
    memset(linedata, 0, DISP_W / 2);
    for (int y = cur_y; y < maxLine; ++y){
        lcd_spi_xfer_line_async(&linedata[0], y, 4);
        lcd_wait_async();
    }    
}

static void baroinfoDraw(int32_t p, int minLine = MIN_LINE_DO, int maxLine = MAX_LINE_DO){
    uint8_t linedata[LINE_BUF_L*3]; //We expose unpacked data in buffer
    int space_above = (maxLine - minLine - CCHAR_H);
    if (space_above > 2) space_above = (space_above + 2)/2;
    
    int start_y = minLine + space_above;
    memset (linedata,0,DISP_W / 2);
    for (int y = minLine; y < start_y; ++y){
        lcd_spi_xfer_line_async(&linedata[0], y, 4);
        lcd_wait_async();
    }
    int cur_y = start_y;
    {
        uint8_t cbuf[CHARS_PER_DISP];
        char sbuf[CHARS_PER_DISP];
        uint32_t t = slowsensorsGetTemperature(FLAG_SENSOR_TEMPERATURE_BAROSENSOR);
        uint32_t alt = getBaroAlt(p, getGlobalReferencePressure(), t);        
        itoSPadded(alt, &sbuf[0], 4, 0);
        sbuf[4] = 'm';
        sbuf[5] = ' ';
        sbuf[6] = ' ';
        itoSPadded(p, &sbuf[7], 6, 4);
        char * str = sbuf;
        while(' '== *str) ++str;

        memset (cbuf, COL2(COL_DCLOCK_FRONT, COL_DCLOCK_BACK), CHARS_PER_DISP);
        int length = strlen(str);
        uint32_t x = (DISP_W - (CCHAR_W)*length) >> 1;
        PRINT_STR_COL_EXTBUF_CL(linedata, x, cur_y, str, cbuf, length);
        cur_y += CCHAR_H;
    }
    memset(linedata, 0, DISP_W / 2);
    for (int y = cur_y; y < maxLine; ++y){
        lcd_spi_xfer_line_async(&linedata[0], y, 4);
        lcd_wait_async();
    }    
}

static uint8_t dclock_last_hour;
static uint8_t dclock_last_min = 1; // This will force redraw when we start at 00:00

bool displayClockTime(bool force){
    uint8_t hour, min, sec;
    uint32_t time_secs_utc = rtc_time_secs_get();
    uint32_t time_secs = rtc_tz_delta(time_secs_utc) + time_secs_utc;
    rtc_secs_to_hms(time_secs, &hour, &min, &sec);
    if ((min != dclock_last_min) || (hour != dclock_last_hour)) force = true;
    if (force){
        updateDayNight();
        dclockDraw(hour, min, rtc_get_dow_from_secs(time_secs), time_secs_utc);
        dclock_last_min = min;
        dclock_last_hour = hour;
    }
    return force;
}

static int32_t dclock_last_pressure;
bool displayClockTimeAndPressure(bool force){
    uint8_t hour, min, sec;
    uint32_t time_secs_utc = rtc_time_secs_get();
    uint32_t time_secs = rtc_tz_delta(time_secs_utc) + time_secs_utc;
    rtc_secs_to_hms(time_secs, &hour, &min, &sec);
    int32_t p = slowsensorsGetPressure(FLAG_SENSOR_PRESSURE_MMHG);
    bool forceClock = false;
    bool forcePressure = false;
    if ((force) || (min != dclock_last_min) || (hour != dclock_last_hour)) forceClock = true;
    if ((force) || (p != dclock_last_pressure)) forcePressure = true;
    int yIntersect = 4 + CCHAR_H + MIN_LINE_DO;
    if (forcePressure){
        baroinfoDraw(p, MIN_LINE_DO, yIntersect);
        dclock_last_pressure = p;
    }
    if (forceClock){
        updateDayNight();
        dclockDraw(hour, min, rtc_get_dow_from_secs(time_secs), time_secs_utc, yIntersect, MAX_LINE_DO);
        dclock_last_min = min;
        dclock_last_hour = hour;
    }
    return force;
}


#define LUX_NAME_SIZE 4
typedef char LuxNameStr_t [LUX_NAME_SIZE];
static const LuxNameStr_t StrLuxName[3] = {{"Lux"},{"kLx"},{"mLx"}};

int luxToStr(int32_t ltd, char *str){
    int dpos = 0;
    int prefnum = 0;
    if (ltd < 1000L){            //under 1 lux
        dpos = 0;
        prefnum = 2;
    } else if(ltd < 10000L){      //under 10 lux
        ltd /= 10;
        dpos = 2;
    } else if(ltd < 100000L){    //under 100 lux
        ltd /= 100;
        dpos = 3;
    } else if(ltd < 10000000L){  //under 10 klux
        ltd /= 1000;
        dpos = 0;
    } else if(ltd < 100000000L){ //under 100 klux
        ltd /= 100000L;
        dpos = 3;
        prefnum = 1;
    } else {
        ltd /= 1000000L;
        dpos = 0;
        prefnum = 1;
    }
    if((ltd >= 0) && (ltd < 10000L)){
        itoSPadded(ltd, str, 4, dpos);
        memcpy(&str[4], (char *) &StrLuxName[prefnum], LUX_NAME_SIZE);
    }else {
        for(dpos = 0; dpos < 7; dpos++) str[dpos]='-';
    }  
    dpos = 7;
    str[dpos] = 0;
    return dpos;
}

#if 0
#define LIGHT_UNIT_LENGTH 5
const char strLightUnitName[LIGHT_UNIT_LENGTH+1] = {"w/cm\x81"};
const uint8_t nanoPrefix[3] = {'n',0x80,'m'}; //0x80 means greec mu

int lightToStr(uint32_t ptd, char *str){
    int dpos;
    int sym_num = 0;   // symbol=nano
    if(ptd < 10000L){            //under 10mw 1234nw/
      // ptd/=1;
      dpos=0;
    }else if(ptd < 100000L){    //under 100uw 12.3uw
        ptd /= 100;
        dpos = 3;
        sym_num = 1;                 //symbol=micro
    }else if(ptd < 10000000L){  //under 10mw 1234uw
        ptd /= 1000;
        dpos = 0;
        sym_num = 1;
    }else if(ptd < 100000000L){ //under 100mw 12.3mw
        ptd /= 100000L;
        dpos = 3;
        sym_num = 2;                 //symbol=mili
    }  else {                 //under 10w  1234mw
        ptd /= 1000000L;
        dpos = 0;
        sym_num = 2;
    }
    if(ptd > 0){  //usedge of TMO instead of validity removes flickering while sensivity adjustments
        itoSPadded(ptd, str, 4, dpos);
        str[4] = nanoPrefix[sym_num];
        memcpy(&str[5], (char *) &strLightUnitName, LIGHT_UNIT_LENGTH);
    }else {
      for(int pos = 0; pos < 5 + LIGHT_UNIT_LENGTH; pos++) str[pos]='^';
    }  
    dpos = 5 + LIGHT_UNIT_LENGTH;
    str[dpos] = 0; 
    return dpos;
}
#else
#define LIGHT_LENGTH 7
int lightToStr(uint32_t ptd, char *str){
    int exp = -7;
    if (ptd > 0){
        while ((exp != 0) && (ptd > 1000)){
            ++exp;
            ptd /= 10;
        }
    }

    if(ptd > 0){  //usedge of TMO instead of validity removes flickering while sensivity adjustments
        itoSPadded(ptd, str, 4, 2);
        str[4] = 'E';
        itoSPadded(exp, &str[5], 2, 0);
    }else {
      for(int pos = 0; pos < LIGHT_LENGTH; pos++) str[pos]='^';
    }  
    return LIGHT_LENGTH;
}

#endif
static inline uint32_t get_lux_bar_length(uint32_t max, uint32_t cur, uint32_t xl){
    if (max > 0x3fffff){
        max >>= 10;
        cur >>= 10;
    }
    uint32_t l = DISP_W - LIGHT_BAR_GAP - xl;
    l = (l*cur)/max;
    if (l < 1) l = 1;
    return l;
}

static inline void put_lux_bar(uint8_t * linedata, int x, int y, int len, uint8_t fcol, uint8_t lcol, uint8_t bcol){
    int ofs = x >> 1;
    uint8_t cbyte = 0;
    for (int d = 0; d < DISP_W - x; ++d){
        uint8_t ccol = bcol;
        if ((1 <= y) && (CHAR_H - 2 >= y)) {
            if ((((1 == y) || (CHAR_H - 2 == y)) && (d <= len)) || (0 == d) || (len == d)) {
                ccol = lcol;
            } else if ((d > 0) && (d < len)) {
                ccol = fcol;
            }
        }
        cbyte = (cbyte << 4) + (ccol << 1);
        if (d & 1) linedata[ofs++] = cbyte;
    }
}

#define PWR_CHANS 5

bool displayLuxInfo(bool force){
static uint32_t last_lux = 0;
static uint32_t last_pwr[PWR_CHANS] = {0};
static const uint8_t chan_cols[PWR_CHANS] = {
    COL2(COL_3_BLACK, COL_3_WHITE),
    COL2(COL_3_BLACK, COL_3_RED),
    COL2(COL_3_BLACK, COL_3_GREEN),
    COL2(COL_3_BLACK, COL_3_CYAN),
    COL2(COL_3_YELLOW, COL_3_BLACK)
};
    char sbuf[CHARS_PER_DISP];         // string print buffer
    uint8_t cbuf[CHARS_PER_DISP];      // color print buffer
    uint8_t linedata[LINE_BUF_L];      // line buffer
    bool updated = force;
    int cur_y = MIN_LINE_DO;
    int base_x = CCHAR_W >> 1;
    if (force){
        brline(linedata, cur_y++, COL2(COL_3_BLACK, COL_3_BLACK), LINE_BUF_L);
        brline(linedata, cur_y++, COL2(COL_INFOBAR_BACK, COL_INFOBAR_BACK), LINE_BUF_L);
    } else {
        cur_y += 2;
    }
    uint32_t lux = slowsensorsGetLux(0, 0) / 10;
    if (force || (lux != last_lux)){
        last_lux = lux;
        memset(linedata, COL2(COL_INFOBAR_BACK, COL_INFOBAR_BACK),LINE_BUF_L);
        int l = luxToStr(lux, sbuf);
        memset(cbuf, COL2(COL_INFOBAR_FRONT, COL_INFOBAR_BACK), sizeof(cbuf));
        PRINT_STR_COL_EXTBUF_CL(linedata, base_x, cur_y, sbuf, cbuf, l);
        updated = true;
    }
    cur_y += CCHAR_H;

    bool colUpd = false;
    if (slowsensorsLuxInSync()){
        uint32_t pwr[PWR_CHANS];
        uint32_t max_pwr = 1;
        for (int chan = 0; chan < PWR_CHANS; ++chan){
            pwr[chan] = slowsensorsGetLux(FLAG_SENSOR_LUX_RAW_NW + chan, 0);
            if (pwr[chan] != last_pwr[chan]) colUpd = true;
            if (pwr[chan] > max_pwr) max_pwr = pwr[chan];
        }
        for (int chan = 0; chan < PWR_CHANS; ++chan){
            if (force || colUpd){
                last_pwr[chan] = pwr[chan];
                memset(linedata, COL2(COL_INFOBAR_BACK, COL_INFOBAR_BACK),LINE_BUF_L);
                int l = lightToStr(pwr[chan], sbuf);
                for (int y = 0; y < CCHAR_H; ++y){
                    memset(cbuf, chan_cols[chan], sizeof(cbuf));
                    int x = base_x;
                    PUT_STR_COL_EXTBUF_CL(linedata, x, y, sbuf, cbuf, l);
                    linedata[(x >> 1) - 1] = (chan_cols[chan] & 0xf0) + (chan_cols[chan] >> 4);
                    x += l * CCHAR_W;
                    linedata[(x >> 1)] = (chan_cols[chan] & 0xf0) + (chan_cols[chan] >> 4);
                    x += CCHAR_W;
                    uint32_t bar_length = get_lux_bar_length(max_pwr, pwr[chan], x);
                    put_lux_bar(linedata, x, y, bar_length, (chan_cols[chan] >> 5), COL_INFOBAR_FRONT, COL_INFOBAR_BACK);
                    lcd_spi_xfer_line_async(&linedata[0], y + cur_y, 4);
                    lcd_wait_async();
                }
                updated = true;
            }
            cur_y += CCHAR_H;
        }
        if (force || colUpd){
            //memset(linedata, COL2(COL_INFOBAR_BACK, COL_INFOBAR_BACK),LINE_BUF_L);
            brline(linedata, cur_y, COL2(COL_INFOBAR_BACK, COL_INFOBAR_BACK), LINE_BUF_L, 2);
            cur_y += 2;
            memset(cbuf, COL2(COL_INFOBAR_FRONT, COL_INFOBAR_BACK), sizeof(cbuf));

            colorxy_t curCol,retCol;
            uint32_t icx,icy;
            float colorTemp;

            slowsensorsGetColorXY(&icx, &icy);
            curCol.x = (float)icx / 65536.0f;
            curCol.y = (float)icy / 65536.0f;
            colorTemp = getColorTemp(curCol);
            retCol = getUVfromColorTemp(colorTemp);
            curCol = getUVfromXY(curCol);
          
            icx = lroundf(1000.0f*curCol.x);
            icy = lroundf(1000.0f*curCol.y);
            if ((icx)>999) icx = 999;
            if ((icy)>999) icy = 999;
            strncpy(sbuf, "U=0.xxx 12345K", sizeof(sbuf));
            itoSPadded(icx, &sbuf[4], 3, 0xff);
            sbuf[7] = ' ';
            if (colorTemp>99999.0f) colorTemp=99999.0f;
            itoSPadded(lroundf(colorTemp), &sbuf[8], 5, 0);
            sbuf[13] = 'K';
            PRINT_STR_COL_EXTBUF_CL(linedata, base_x, cur_y, sbuf, cbuf, strlen(sbuf));
            cur_y += CCHAR_H;            
            strncpy(sbuf, "V=0.xxx 0.xxxG", sizeof(sbuf));
            itoSPadded(icy, &sbuf[4], 3, 0xff);
            sbuf[7] = ' ';
            icx = lroundf(1000.0f*hypotf(curCol.x - retCol.x, curCol.y - retCol.y));
            itoSPadded(icx, &sbuf[10], 3, 0xff);
            if (curCol.y > retCol.y) sbuf[13]='G'; else sbuf[13]='M';
            PRINT_STR_COL_EXTBUF_CL(linedata, base_x, cur_y, sbuf, cbuf, strlen(sbuf));
            cur_y += CCHAR_H;            
        }
    } else {
        cur_y += CCHAR_H * 7;
    }    
    memset(linedata, COL2(COL_INFOBAR_BACK, COL_INFOBAR_BACK),LINE_BUF_L);
    if (force) {
        for (; cur_y < MAX_LINE_DO - 1; ++cur_y){
            lcd_spi_xfer_line_async(&linedata[0], cur_y, 4);
            lcd_wait_async();
        }
        if ( cur_y < MAX_LINE_DO) {
            brline(linedata, cur_y++, COL2(COL_3_BLACK, COL_3_BLACK), LINE_BUF_L);
        }
    }
    return updated;
}

#endif