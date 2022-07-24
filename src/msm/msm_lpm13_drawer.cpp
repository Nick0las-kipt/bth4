#define LOG_LEVEL 4
#include <logging/log.h>
LOG_MODULE_REGISTER(drawer);
#include <logging/log_ctrl.h>
#include "msm_lpm13_drawer.h"
#include "fonts.h"
#include "lpm013m126.h"
#include <string.h>

// drawer supplementary functions

#define COL(col) ((col + (col << 4)) << 1)

#define STEP_X 2

static inline void lcd_PrintStrCol(uint8_t * buffer, uint8_t x, uint8_t line, char *txt, uint8_t fcol, uint8_t bcol,uint8_t llen){
    arr_to_fb_4bit_pl(&buffer[(x >> 1)], txt, llen, line, line, COL(fcol), COL(bcol), 0);
}

// drawer constructor
MSMDrawer::MSMDrawer(uint8_t x,uint8_t y,uint8_t nsym,uint8_t fcol,uint8_t bcol,uint8_t scol,uint8_t sbcol,uint8_t ncol):
                x0(x),y0(y),ncl(nsym),_fcol(fcol),_bcol(bcol),_scol(scol),_sbcol(sbcol),_ncol(ncol){}

// drawer methods
void MSMDrawer::draw(MSMDisplay* disp){
    uint8_t ly=y0;
    uint8_t i,n,h;
    MSMaList * clist;
    MSMItem * citem;
    char sbuf[3+MSM_MENU_STRLENGTH]; // string print buffer
    uint8_t cbuf[3+MSM_MENU_STRLENGTH]; // color print buffer
    uint8_t linedata[LINE_BUF_L]; // line buffer
    h=disp->getHeight();
    memset(linedata,COL(_fcol), sizeof(linedata));
    if (disp->isopen()){
        // Draw title
        clist=disp->getList();
        citem=clist->getTitle();
        if ((disp->getUpdated())||(citem->getUpdated())||((MENU_DELIMETER)&&(disp->getUpdatedAny()))){ // check disp updated
            citem->getName(&sbuf[0],ncl);//get list name
            if (MENU_DELIMETER){
                strFix(&sbuf[0],ncl);
                i=1+disp->getLevel();
                n=ncl-1;
                while ((i>0)&(n>2)){
                    i--;
                    uint8_t cn=1+disp->getPosAtLevel(i);
                    if (cn>=10){
                        sbuf[n--]=0x30+cn%10;
                        cn/=10;
                    }
                    sbuf[n--]=0x30+cn;
                    if (i) sbuf[n--]=MENU_DELIMETER;
                }
            }
            memset(&linedata[x0 >> 1],COL(_bcol), ((CHAR_W*ncl) >> 1) + (STEP_X));
            memset(cbuf,COL2(_fcol,_bcol),sizeof(cbuf));
            print_str_col_extbuf(linedata, x0 + STEP_X, ly, sbuf, cbuf, ncl);
        }
        ly+=CHAR_H;
        // Draw line. Line logically is a part of    title
        if (disp->getUpdated()){ // check disp updated
            memset(&linedata[x0 >> 1],COL(_bcol), ((CHAR_W*ncl) >> 1) + (STEP_X));
            lcd_spi_xfer_line(&linedata[0], ly, 4);
            lcd_spi_xfer_line(&linedata[0], ly+3, 4);
            memset(&linedata[x0 >> 1],COL(_fcol), ((CHAR_W*ncl) >> 1) + (STEP_X));
            lcd_spi_xfer_line(&linedata[0], ly+1, 4);
            lcd_spi_xfer_line(&linedata[0], ly+2, 4);
        }
        ly+=4;
        n=disp->getTop();
        memset(&linedata[x0 >> 1],COL(_bcol), ((CHAR_W*ncl) >> 1) + (STEP_X));
        for (i=0;i<h;++i){
            citem=clist->getItem(n);
            bool doPrint = 0;
            memset(cbuf,COL2(_fcol,_bcol),sizeof(cbuf));
            if(citem!=0){//item defined
                if ((disp->getUpdated(n))||(clist->getUpdated(n))||(citem->getUpdated())){ //check if updated
                    citem->getName(&sbuf[0],ncl);//get item name
                    if (n==disp->getPos()){//item selected
                        uint8_t p1 = 0;
                        uint8_t p2 = ncl - 1;
                        if (disp->itemopen()){//item os open
                            citem->getSelection(p1,p2,ncl);
                        }
                        memset(&cbuf[p1],COL2(_scol,_sbcol),p2-p1+1);
                    }
                    doPrint = 1;
                    // end selected
                } else { //not updated
                }// end updated
            } else {// item is null
                if ((disp->getUpdated(n))||(clist->getUpdated(n))){ //check if updated
                    sbuf[0]=0;
                    if (n==disp->getPos()){//item selected
                        memset(cbuf,COL2(_scol,_sbcol),sizeof(cbuf));
                    }// end selected
                    doPrint = 1;
                } else { // not updated
                }
            }//end defined
            if (doPrint) print_str_col_extbuf(linedata, x0 + STEP_X, ly, sbuf, cbuf, ncl);
            n++;
            ly+=CHAR_H;
        }//end cycle
        //clear all updated flags for display and list
        clist->clearUpdated();
        disp->clearUpdated();
    }else{ //not open
        h+=1;
        if (disp->getUpdated()) {
            for (int l = 0; l < h*CHAR_H+4; ++l) lcd_spi_xfer_line(&linedata[0], l+y0, 4);
            disp->clearUpdated();
        }
    }
}

