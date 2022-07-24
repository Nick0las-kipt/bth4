// Minimalistic Smart Meun item template classes

#ifndef __MSMTEMPLATES_H_
#define __MSMTEMPLATES_H_ 1

#include "msm.h"
#include "strservice.h"

// ----------------------
// msmparam - template for parameter class
// ----------------------

template<typename Tparam, uint8_t nch, uint8_t dpos>
class MSMintparam:public IOpenItem{
public:
  MSMintparam(Tparam minVal,Tparam maxVal,Tparam Val):_minVal(minVal),_maxVal(maxVal),_corVal(Val),_actVal(Val){}

  void setMin(Tparam val){
    _minVal=val;
  }
  void setMax(Tparam val){
    _maxVal=val;
  }

  Tparam getMin(){
    return _minVal;
  }
  Tparam getMax(){
    return _maxVal;
  }

  // using this method will not involve update flag hierarchy. Use only before show.
  void setVal(Tparam val){
    _corVal=val;
    _actVal=val;
  }

  Tparam getVal(){
    return _actVal;
  }

  // redefine this to process changes by click
  virtual void onChange(){
  }

  // IOpenItem methods
  // start editing
  virtual  void forceOpen(){
      _corVal=_actVal;
  }
  
  // pass keys ruturn flag if still open
  virtual bool passKeys(uint8_t key,uint8_t count){
    if((key==MSM_KEY_ENTER)&&(count==0)){
      _corVal=_actVal;
      onChange();
      return false;
    }
    if(key==MSM_KEY_ESC){
      _actVal=_corVal;
      return false;
    }
    Tparam delta;
    convertCount(count,delta);
//    Tparam delta=1;
//    if (count>0){
//      delta=count-1;
//      if ((sizeof(delta)>1)&&(count>4)) delta*=(delta-2);
//      if ((sizeof(delta)>2)&&(count>5)) delta*=(Tparam)(count-5);
//    }
    if(key==MSM_KEY_UP){
      if((_maxVal-delta)>_actVal) _actVal+=delta;else _actVal=_maxVal;
    }
    if(key==MSM_KEY_DOWN){
      if(_actVal>_minVal+delta) _actVal-=delta;else _actVal=_minVal;
    }
    return true;
  }

//  // dicard all editing changes and close editing
//  virtual void forceClose(){
//    _actVal=_corVal;
//  }
// 
//  // get flag if opened
//  virtual bool isOpened(){
//    return (_actVal!=_corVal);
//  }

  // get contents length
  virtual uint8_t getLength(){
    return nch;
  }
 
  // get first and last positions of parameter edited (returns 0, length-1 for simple objects)
  virtual void getSelPos(uint8_t &firstp, uint8_t &lastp){
    firstp=0;
    lastp=nch-1;
  }
 
  //convert to string
  virtual void toStr(char* str){
    itoSPadded(_actVal,str,nch,dpos);
  }

protected:
  Tparam _minVal,_maxVal,_corVal,_actVal;
};

// ----------------------
// msmintitemC - template for item containing an integer with update call
// ----------------------


template<typename Tparam, uint8_t nch, uint8_t dpos>
class MSMintitemC:public MSMItem{
  public:
  MSMintitemC(const char* name,uint8_t newflags,Tparam minVal,Tparam maxVal,Tparam Val,void (*func) (Tparam)) : MSMItem(newflags,name,(void*)func),_minVal(minVal),_maxVal(maxVal),_corVal(Val),_actVal(Val){}

  virtual void callUpdate(){
      if (_rptr!=0){
        void (*func)(Tparam)= (void(*)(Tparam))_rptr;
        (*func)(_actVal);
      }  
  }
  
  virtual void getName(char* str,uint8_t lLim){
    loadName(str,lLim);
    strFix(str,lLim);
    if (lLim>=nch) itoSPadded(_actVal,&str[lLim-nch],nch,dpos);
  }
  virtual void Enter(){
    _corVal=_actVal;
    setUpdated();
  }
  virtual bool passKeys(uint8_t key,uint8_t count){
    setUpdated();
    if((key==MSM_KEY_ENTER)&&(count==0)){
      _corVal=_actVal;
      callUpdate();
      return false;
    }
    if(key==MSM_KEY_ESC){
      _actVal=_corVal;
      return false;
    }
    Tparam delta;
    convertCount(count,delta);
//    Tparam delta=1;
//    if (count>0){
//      delta=count-1;
//      if ((sizeof(delta)>1)&&(count>4)) delta*=(delta-2);
//      if ((sizeof(delta)>2)&&(count>5)) delta*=(Tparam)(count-5);
//    }
    
    if(key==MSM_KEY_UP){
      if((_maxVal-delta)>_actVal) _actVal+=delta;else _actVal=_maxVal;
    }
    if(key==MSM_KEY_DOWN){
      if(_actVal>_minVal+delta) _actVal-=delta;else _actVal=_minVal;
    }
    return true;
  }
  virtual void getSelection(uint8_t &firstp, uint8_t &lastp,uint8_t lLim){
    firstp=lLim-nch;
    lastp=lLim-1;
  }

  void setMin(Tparam val){
    _minVal=val;
    if (_maxVal<val)_maxVal=val;
    if (_actVal<val){
      _actVal=val;
      callUpdate();
      setUpdated();
    }
  }
  void setMax(Tparam val){
    _maxVal=val;
    if (_minVal>val)_minVal=val;
    if (_actVal>val){
      _actVal=val;
      callUpdate();
      setUpdated();
    }    
  }

  Tparam getMin(){
    return _minVal;
  }
  Tparam getMax(){
    return _maxVal;
  }

  void setVal(Tparam val){
    if (_maxVal<val)_maxVal=val;
    if (_minVal>val)_minVal=val;
    _corVal=val;
    _actVal=val;
    callUpdate();
    setUpdated();
  }

  void setValSilent(Tparam val){
    if (_maxVal<val)_maxVal=val;
    if (_minVal>val)_minVal=val;
    _corVal=val;
    _actVal=val;
    // callUpdate(); //no callUpdate in silent
    setUpdated();
  }


  Tparam getVal(){
    return _actVal;
  }

protected:
  Tparam _minVal,_maxVal,_corVal,_actVal;
};

template<typename Tparam>
class MSMintitemCG:public MSMItem{
  private:
void fetchActVal(){
    if (_rptr!=0){
      Tparam (*func)(Tparam*) = (Tparam(*)(Tparam*))_rptr;
      Tparam tmpVal = (*func)(NULL);
      _actVal = tmpVal;
    }  
  }

  void fetchActValIfNeeded(){
    if (!_open) fetchActVal();
  }

  public:
  MSMintitemCG(const char* name,uint8_t newflags,Tparam minVal,Tparam maxVal,Tparam (*func) (Tparam*),  uint8_t nch, uint8_t dpos = 0) :
   MSMItem(newflags,name,(void*)func),_minVal(minVal),_maxVal(maxVal),_nch(nch),_dpos(dpos),_open(false){}

  virtual void callUpdate(){
    if (_rptr!=0){
      Tparam (*func)(Tparam*) = (Tparam(*)(Tparam*))_rptr;
      Tparam tmpVal = (*func)(&_actVal);
      _actVal = tmpVal;
    }  
  }
  
  virtual void getName(char* str,uint8_t lLim){
    fetchActValIfNeeded();
    loadName(str,lLim);
    strFix(str,lLim);
    if (lLim>=_nch) itoSPadded(_actVal,&str[lLim-_nch],_nch,_dpos);
  }
  virtual void Enter(){
    fetchActValIfNeeded();
    _open = true;
    setUpdated();
  }
  virtual bool passKeys(uint8_t key,uint8_t count){
    setUpdated();
    if((key==MSM_KEY_ENTER)&&(count==0)){
      callUpdate();
      _open = false;
      return _open;
    }
    if(key==MSM_KEY_ESC){
      _open = false;
      return _open;
    }
    Tparam delta;
    convertCount(count,delta);
    
    if(key==MSM_KEY_UP){
      if((_maxVal-delta)>_actVal) _actVal+=delta;else _actVal=_maxVal;
    }
    if(key==MSM_KEY_DOWN){
      if(_actVal>_minVal+delta) _actVal-=delta;else _actVal=_minVal;
    }
    return _open;
  }

  virtual void getSelection(uint8_t &firstp, uint8_t &lastp,uint8_t lLim){
    firstp=lLim-_nch;
    lastp=lLim-1;
  }

  void setMin(Tparam val){
    fetchActValIfNeeded();
    _minVal=val;
    if (_maxVal<val)_maxVal=val;
    if (_actVal<val){
      _actVal=val;
      callUpdate();
      setUpdated();
    }
  }

  void setMax(Tparam val){
    fetchActValIfNeeded();
    _maxVal=val;
    if (_minVal>val)_minVal=val;
    if (_actVal>val){
      _actVal=val;
      callUpdate();
      setUpdated();
    }    
  }

  Tparam getMin(){
    return _minVal;
  }
  Tparam getMax(){
    return _maxVal;
  }

  void setVal(Tparam val){
    if (_maxVal<val)_maxVal=val;
    if (_minVal>val)_minVal=val;
    _actVal=val;
    callUpdate();
    setUpdated();
  }


  Tparam getVal(){
    fetchActValIfNeeded();
    return _actVal;
  }

protected:
  Tparam _minVal,_maxVal,_corVal,_actVal;
  uint8_t _nch, _dpos;
  bool _open;
};


// ----------------------
// msmintitemCP - template for item containing pointer to an integer with update call
// ----------------------


template<typename Tparam, uint8_t nch, uint8_t dpos>
class MSMintitemCP:public MSMItem{
  public:
  MSMintitemCP(const char* name,uint8_t newflags,Tparam minVal,Tparam maxVal,Tparam * val,void (*func) (Tparam)) : MSMItem(newflags,name,(void*)func),_minVal(minVal),_maxVal(maxVal),_corVal(*val),_actVal(val){}

  virtual void callUpdate(){
      if (_rptr!=0){
        void (*func)(Tparam)= (void(*)(Tparam))_rptr;
        (*func)(*_actVal);
      }  
  }
  
  virtual void getName(char* str,uint8_t lLim){
    loadName(str,lLim);
    strFix(str,lLim);
    if (lLim>=nch) itoSPadded(*_actVal,&str[lLim-nch],nch,dpos);
  }
  virtual void Enter(){
    _corVal=*_actVal;
    setUpdated();
  }
  virtual bool passKeys(uint8_t key,uint8_t count){
    setUpdated();
    if((key==MSM_KEY_ENTER)&&(count==0)){
      _corVal=*_actVal;
      callUpdate();
      return false;
    }
    if(key==MSM_KEY_ESC){
      *_actVal=_corVal;
      return false;
    }
    Tparam delta;
    convertCount(count,delta);
//    Tparam delta=1;
//    if (count>0){
//      delta=count-1;
//      if ((sizeof(delta)>1)&&(count>4)) delta*=(delta-2);
//      if ((sizeof(delta)>2)&&(count>5)) delta*=(Tparam)(count-5);
//    }
    
    if(key==MSM_KEY_UP){
      if((_maxVal-delta)>*_actVal) *_actVal+=delta;else *_actVal=_maxVal;
    }
    if(key==MSM_KEY_DOWN){
      if(*_actVal>_minVal+delta) *_actVal-=delta;else *_actVal=_minVal;
    }
    return true;
  }
  virtual void getSelection(uint8_t &firstp, uint8_t &lastp,uint8_t lLim){
    firstp=lLim-nch;
    lastp=lLim-1;
  }

  void setMin(Tparam val){
    _minVal=val;
    if (_maxVal<val)_maxVal=val;
    if (*_actVal<val){
      *_actVal=val;
      callUpdate();
      setUpdated();
    }
  }
  void setMax(Tparam val){
    _maxVal=val;
    if (_minVal>val)_minVal=val;
    if (*_actVal>val){
      *_actVal=val;
      callUpdate();
      setUpdated();
    }    
  }

  Tparam getMin(){
    return _minVal;
  }
  Tparam getMax(){
    return _maxVal;
  }

  void setVal(Tparam val){
    if (_maxVal<val)_maxVal=val;
    if (_minVal>val)_minVal=val;
    _corVal=val;
    *_actVal=val;
    callUpdate();
    setUpdated();
  }

  void setValSilent(Tparam val){
    if (_maxVal<val)_maxVal=val;
    if (_minVal>val)_minVal=val;
    _corVal=val;
    *_actVal=val;
    // callUpdate(); //no callUpdate in silent
    setUpdated();
  }


  Tparam getVal(){
    return *_actVal;
  }

protected:
  Tparam _minVal,_maxVal,_corVal;
  Tparam * _actVal;
};

// ----------------------
// msmintitemP - template for item containing pointer to an integer
// ----------------------


template<typename Tparam, uint8_t nch, uint8_t dpos>
class MSMintitemP:public MSMItem{
  public:
  MSMintitemP(const char* name,uint8_t newflags,Tparam minVal,Tparam maxVal,Tparam * val) : MSMItem(newflags,name,(void*)val),_minVal(minVal),_maxVal(maxVal),_corVal(*val){}

  virtual void getName(char* str,uint8_t lLim){
    loadName(str,lLim);
    strFix(str,lLim);
    if (lLim>=nch) itoSPadded(*((Tparam*)_rptr),&str[lLim-nch],nch,dpos);
  }
  virtual void Enter(){
    _corVal=*((Tparam*)_rptr);
    setUpdated();
  }
  virtual bool passKeys(uint8_t key,uint8_t count){
    setUpdated();
    if((key==MSM_KEY_ENTER)&&(count==0)){
      _corVal=*((Tparam*)_rptr);
      return false;
    }
    if(key==MSM_KEY_ESC){
      *((Tparam*)_rptr)=_corVal;
      return false;
    }
    Tparam delta;
    convertCount(count,delta);
//    Tparam delta=1;
//    if (count>0){
//      delta=count-1;
//      if ((sizeof(delta)>1)&&(count>4)) delta*=(delta-2);
//      if ((sizeof(delta)>2)&&(count>5)) delta*=(Tparam)(count-5);
//    }
    
    if(key==MSM_KEY_UP){
      if((_maxVal-delta)>*((Tparam*)_rptr)) *((Tparam*)_rptr)+=delta;else *((Tparam*)_rptr)=_maxVal;
    }
    if(key==MSM_KEY_DOWN){
      if(*((Tparam*)_rptr)>_minVal+delta) *((Tparam*)_rptr)-=delta;else *((Tparam*)_rptr)=_minVal;
    }
    return true;
  }
  virtual void getSelection(uint8_t &firstp, uint8_t &lastp,uint8_t lLim){
    firstp=lLim-nch;
    lastp=lLim-1;
  }

  void setMin(Tparam val){
    _minVal=val;
    if (_maxVal<val)_maxVal=val;
    if (*((Tparam*)_rptr)<val){
      *((Tparam*)_rptr)=val;
      setUpdated();
    }
  }
  void setMax(Tparam val){
    _maxVal=val;
    if (_minVal>val)_minVal=val;
    if (*((Tparam*)_rptr)>val){
      *((Tparam*)_rptr)=val;
      setUpdated();
    }    
  }

  Tparam getMin(){
    return _minVal;
  }
  Tparam getMax(){
    return _maxVal;
  }

  void setVal(Tparam val){
    if (_maxVal<val)_maxVal=val;
    if (_minVal>val)_minVal=val;
    _corVal=val;
    *((Tparam*)_rptr)=val;
    setUpdated();
  }


  Tparam getVal(){
    return *((Tparam*)_rptr);
  }

protected:
  Tparam _minVal,_maxVal,_corVal;
};


// ----------------------
// msmnameditemP - template for item containing pointer to an integer name index
// ----------------------
template <typename Tparam, uint8_t nch>
class MSMnameditemP: public MSMintitemP<Tparam, nch, 0> {
  public:
  MSMnameditemP(const char* name,uint8_t newflags,Tparam * val, char ** nameTable): MSMintitemP<Tparam, nch, 0>(name, newflags, 0, 0, val), _nameTable(nameTable){
    Tparam newMax = 0;
    while (_nameTable[newMax]) ++newMax;
    this->_maxVal = newMax - 1;
    //setMax(newMax - 1);
  }

  virtual void getName(char* str,uint8_t lLim){
    this->loadName(str,lLim);
    strFix(str,lLim);
    if (lLim>=nch) {
      Tparam val = *((Tparam*)this->_rptr);
      if (this->ramName()){
        strCpyL(&str[lLim-nch],_nameTable[val],nch);
      }else{
        pgmLoadStr(&str[lLim-nch],_nameTable[val],nch);
      }
      strFix(&str[lLim-nch],nch);
    }
  }

protected:
  char ** _nameTable;
};

// ----------------------
// msmnameditemC - template for item containing call to an integer dislpayed name
// ----------------------
template <typename Tparam, uint8_t nch>
class MSMnameditemC: public MSMintitemC<Tparam, nch, 0> {
  public:
  MSMnameditemC(const char* name,uint8_t newflags,void (*func) (Tparam), const char * const * nameTable): MSMintitemC<Tparam, nch, 0>(name, newflags, 0, 0, 0, func), _nameTable(nameTable){
    Tparam newMax = 0;
    while (_nameTable[newMax]) ++newMax;
    this->_maxVal = newMax - 1;
    //setMax(newMax - 1);
  }

  virtual void getName(char* str,uint8_t lLim){
    this->loadName(str,lLim);
    strFix(str,lLim);
    if (lLim>=nch) {
      Tparam val = this->_actVal;
      if (this->ramName()){
        strCpyL(&str[lLim-nch],_nameTable[val],nch);
      }else{
        pgmLoadStr(&str[lLim-nch],_nameTable[val],nch);
      }
      strFix(&str[lLim-nch],nch);
    }
  }

protected:
  const char * const * _nameTable;
};

template <typename Tparam>
class MSMnameditemCG: public MSMintitemCG<Tparam> {
  public:
  MSMnameditemCG(const char* name,uint8_t newflags, uint8_t nch, Tparam (*func) (Tparam*), const char * const * nameTable): MSMintitemCG<Tparam>(name, newflags, 0, 0, func, nch, 0), _nameTable(nameTable){
    Tparam newMax = 0;
    while (_nameTable[newMax]) ++newMax;
    this->_maxVal = newMax - 1;
    //setMax(newMax - 1);
  }

  virtual void getName(char* str,uint8_t lLim){
    this->loadName(str,lLim);
    strFix(str,lLim);
    uint8_t _nch = this->_nch;
    if (lLim >= _nch) {
      Tparam val = this->_actVal;
      if (this->ramName()){
        strCpyL(&str[lLim-_nch],_nameTable[val],_nch);
      }else{
        pgmLoadStr(&str[lLim-_nch],_nameTable[val],_nch);
      }
      strFix(&str[lLim-_nch],_nch);
    }
  }

protected:
  const char * const * _nameTable;
};


// Days in month, AVR specific (pgmspace used)
// const uint8_t TabDaysInMonth[12] PROGMEM ={31,28, 31,30,31, 30,31,31, 30,31,30, 31};

// uint8_t DaysInMonth(uint8_t Month,uint8_t Year){
//  uint8_t i=pgm_read_byte(&TabDaysInMonth[Month-1]);
//  if((0==(Year&0x03))&&(Month==2))  i++;
//  return i;
//}


//-------------------------
// struct for time storage and convertion to string
// contains array of 3 uint8_t - hours,minutes and seconds respectively 
//-------------------------

struct TimeArr{
 uint8_t data[3];
 TimeArr(uint8_t nhour,uint8_t nmin, uint8_t nsec){
    data[0]=nhour%24;
    data[1]=nmin%60;
    data[2]=nsec%60;
 }
 TimeArr(){
    data[0]=0;
    data[1]=0;
    data[2]=0;
 }
 void timeToStr(char* str){
      itoSPadded(data[0],&str[0],2,0xff);  
      str[2]=MSM_TIME_SEPARATOR;
      itoSPadded(data[1],&str[3],2,0xff);
      str[5]=MSM_TIME_SEPARATOR;
      itoSPadded(data[2],&str[6],2,0xff);               
  }
  bool incTime(uint8_t sectoadd){
    data[2]+=sectoadd;
    if (data[2]>=60){//sec overflow
      data[1]+=data[2]/60;
      data[2]=data[2]%60;
      if (data[1]>=60){//min overflow
        data[0]+=data[1]/60;
        data[1]=data[1]%60;
        if (data[0]>=24){//hour overflow
          data[0]=data[0]%24;
          return true;
        }//end hour overflow
      }//end min overflow
    }//end sec overflow
    return false;
  }
  void addVal(uint8_t epos,int8_t delta){
    uint8_t vmax=24;
    if (epos>2) return;
    if (epos) vmax=60;
    if(delta>=0){
      if (delta>(vmax>>1)) delta=(vmax>>1);
    }else{
      if (delta<-(vmax>>1)) delta=-(vmax>>1);      
      delta+=vmax;
    }  
    data[epos] =(data[epos]+delta)%vmax;
  }
  bool operator==(const TimeArr & other) const {
    if ((other.data[0]==data[0])&&(other.data[1]==data[1])&&(other.data[2]==data[2]))return true; else return false;
  }
  
  int32_t nsec(){
    return (data[0]*60L+data[1])*60L+data[2];
  }
};

//-------------------------
// struct for date storage and conversion to string
// contains array of 3 uint8_t
// which of them are DD,MM, and YY is determined by MSM_IDAY, MSM_MONTH, and MSM_YEAR constants
//-------------------------
struct DateArr{
 uint8_t data[3];
// Days in month, universal 
static uint8_t daysInMonth(uint8_t month,uint8_t year){
  uint8_t i=31;
  if ((month==4)||(month==6)||(month==9)||(month==11)) i=30;
  if (month==2){ if (year&0x03) i=28; else i=29;}
  return i;
}

static void limitDay(uint8_t &day,uint8_t month,uint8_t year){
  uint8_t lday=daysInMonth(month,year);
  if (day>lday)day=lday;
}

 
DateArr(uint8_t nday, uint8_t nmonth, uint8_t nyear){
    data[MSM_IDAY]=nday;
    data[MSM_IMONTH]=nmonth;
    data[MSM_IYEAR]=nyear;
    limitDay(data[MSM_IDAY],data[MSM_IMONTH],data[MSM_IYEAR]);
 }
 
DateArr(){
    data[MSM_IDAY]=1;
    data[MSM_IMONTH]=1;
    data[MSM_IYEAR]=0;
 }
 
void dateToStr(char* str){
      itoSPadded(data[0],&str[0],2,0xff);  
      str[2]=MSM_DATE_SEPARATOR;
      itoSPadded(data[1],&str[3],2,0xff);
      str[5]=MSM_DATE_SEPARATOR;
      itoSPadded(data[2],&str[6],2,0xff);
}
  
bool incDate(uint8_t daystoadd){
    data[MSM_IDAY]+=daystoadd;
    if (data[MSM_IDAY]>daysInMonth(data[MSM_IMONTH],data[MSM_IYEAR])){//day overflow
      data[MSM_IDAY]-=daysInMonth(data[MSM_IMONTH],data[MSM_IYEAR]);
      data[MSM_IMONTH]++;
      if (data[MSM_IMONTH]>12){//month overflow
        data[MSM_IMONTH]=1;
        data[MSM_IYEAR]++;
        if (data[MSM_IYEAR]>99){//year overflow
          data[MSM_IYEAR]=0;
          return true;
        }//end year overflow
      }//end month overflow
    }//end day overflow
  return false;
  }
  void addVal(uint8_t epos, int8_t delta){
    uint8_t vrange=100;
    uint8_t vmin=0;
    if (epos>2) return;
    if (epos==MSM_IDAY){vrange=31;vmin=1;}else
    if (epos==MSM_IMONTH){vrange=12;vmin=1;};
    if (delta>=0){
      if (delta>(vrange>>1)) delta=(vrange>>1);
    }else{
      if (delta<-(vrange>>1)) delta=-(vrange>>1);
      delta+=vrange;
    }
    data[epos] =vmin+(data[epos]+delta-vmin)%vrange;  
  }
  bool operator==(const DateArr & other) const {
    if ((other.data[0]==data[0])&&(other.data[1]==data[1])&&(other.data[2]==data[2]))return true; else return false;
  }
};


// ----------------------
// msmtimeitemC - item containing time with update call
// ----------------------
class MSMtimeitemC:public MSMItem{
  public:

  MSMtimeitemC(const char* name,uint8_t newflags,uint8_t nhour,uint8_t nmin, uint8_t nsec,void (*func) (void)) : MSMItem(newflags,name,(void*)func),cur(nhour,nmin,nsec),backup(cur),epos(-1){}
  
  virtual void callUpdate(){
    if (_rptr!=0){
      void (*func)(void)= (void(*)(void))_rptr;
      (*func)();
    }  
  }

 bool editInProgress(){
   return (epos>=0);
 }

 void timeToStr(char* str,bool ExplicitF=true){
   if ((ExplicitF)||(epos<0)) cur.timeToStr(str);else backup.timeToStr(str);
  }
  
  virtual void getName(char* str,uint8_t lLim){
    loadName(str,lLim);
    strFix(str,lLim);
    if (lLim>=8){
      timeToStr(&str[lLim-8]);
    }
  }
  virtual void Enter(){
    epos=0;
    backup=cur;
    setUpdated();
  }
  virtual bool passKeys(uint8_t key,uint8_t count){
    setUpdated();
    if((key==MSM_KEY_ENTER)&&(count==0)){
    if(epos==2){
      backup=cur;
      epos=-1;
      callUpdate();
      return false;
    }else epos++;
    }
    if(key==MSM_KEY_ESC){
    cur=backup;
    epos=-1;
    return false;
    }
    if(key==MSM_KEY_LEFT){
      if(epos) epos--;
      return true;
    }
    if(key==MSM_KEY_RIGHT){
      if(epos<2) epos++;
      return true;
    }
    if (count) count=(count+2)>>2; else count=1;
    if (key==MSM_KEY_UP) cur.addVal(epos,count);else if (key==MSM_KEY_DOWN) cur.addVal(epos,-count);
    return true;
  }

  virtual void getSelection(uint8_t &firstp, uint8_t &lastp,uint8_t lLim){
    firstp=lLim-8+3*epos;
    lastp=firstp+1;
  }
  
  // if ExplicitF=true returns the real displayed value cur
  // if ExplicitF=false returns last approved value backup
  
  void getTime(uint8_t &ghour,uint8_t &gmin,uint8_t &gsec,bool ExplicitF=false){
    TimeArr src=getTimeArr(ExplicitF);
    ghour=src.data[0];
    gmin=src.data[1];
    gsec=src.data[2];
  }

  uint8_t getHour(bool ExplicitF=false){
    TimeArr src=getTimeArr(ExplicitF);
    return src.data[0];
  }

  uint8_t getMin(bool ExplicitF=false){
    TimeArr src=getTimeArr(ExplicitF);
    return src.data[1];
  }

  uint8_t getSec(bool ExplicitF=false){
    TimeArr src=getTimeArr(ExplicitF);
    return src.data[2];
  }

  TimeArr getTimeArr(bool ExplicitF=false){
    if ((ExplicitF)||(epos<0))return cur;else return backup;
  }

  void setTimeArr(TimeArr newtime,bool alert=false){
    cur=newtime;
    setUpdated();
    if (alert) callUpdate();
  }


  void setTime(uint8_t nhour,uint8_t nmin,uint8_t nsec,bool alert=false){
      setTimeArr(TimeArr(nhour,nmin,nsec),alert);
  }
  
  bool incTime(uint8_t sectoadd,bool alert=false){
    TimeArr temp;
    bool foverflow=false;
    if (sectoadd){
      setUpdated();
      if (epos>=0) foverflow=backup.incTime(sectoadd); else {
        foverflow=cur.incTime(sectoadd);
        if (alert) callUpdate();
      }
    }
    return foverflow;
  }
  
  protected:
  TimeArr cur,backup;
  int8_t epos;
};


// ----------------------
// msmdateitemC - item containing date with update call
// ----------------------
class MSMdateitemC:public MSMItem{
  public:

  MSMdateitemC(const char* name,uint8_t newflags,uint8_t nday,uint8_t nmonth, uint8_t nyear,void (*func) (void)) : MSMItem(newflags,name,(void*)func),cur(nday,nmonth,nyear),backup(cur),epos(-1){}
  
  virtual void callUpdate(){
    if (_rptr!=0){
      void (*func)(void)= (void(*)(void))_rptr;
      (*func)();
    }  
  }
 bool editInProgress(){
   return (epos>=0);
 }

void dateToStr(char* str,bool ExplicitF=true){
   if ((ExplicitF)||(epos<0)) cur.dateToStr(str);else backup.dateToStr(str);
}
  
  virtual void getName(char* str,uint8_t lLim){
    loadName(str,lLim);
    strFix(str,lLim);
    if (lLim>=8){
      dateToStr(&str[lLim-8]);
    }
  }
  virtual void Enter(){
    epos=0;
    backup=cur;
    setUpdated();
  }
  virtual bool passKeys(uint8_t key,uint8_t count){
    setUpdated();
    if((key==MSM_KEY_ENTER)&&(count==0)){
    if(epos==2){
      DateArr::limitDay(cur.data[MSM_IDAY],cur.data[MSM_IMONTH],cur.data[MSM_IYEAR]);
      backup=cur;
      epos=-1;
      callUpdate();
      return false;
    }else epos++;
    }
    if(key==MSM_KEY_ESC){
    cur=backup;
    epos=-1;
    return false;
    }
    if(key==MSM_KEY_LEFT){
      if(epos) epos--;
      return true;
    }
    if(key==MSM_KEY_RIGHT){
      if(epos<2) epos++;
      return true;
    }
    if (count) count=(count+2)>>2; else count=1; 
    if (key==MSM_KEY_UP) cur.addVal(epos,count);else if (key==MSM_KEY_DOWN) cur.addVal(epos,-count);
    return true;
  }

  virtual void getSelection(uint8_t &firstp, uint8_t &lastp,uint8_t lLim){
    firstp=lLim-8+3*epos;
    lastp=firstp+1;
  }
  
  void getDate(uint8_t &gday,uint8_t &gmonth,uint8_t &gyear,bool ExplicitF=false){
    DateArr src=getDateArr(ExplicitF);
    gday=src.data[MSM_IDAY];
    gmonth=src.data[MSM_IMONTH];
    gyear=src.data[MSM_IYEAR];
    DateArr::limitDay(gday,gmonth,gyear);
  }

  uint8_t getYear(bool ExplicitF=false){
    DateArr src=getDateArr(ExplicitF);
    return src.data[MSM_IYEAR];
  }

  uint8_t getMonth(bool ExplicitF=false){
    DateArr src=getDateArr(ExplicitF);
    return src.data[MSM_IMONTH];
  }

  uint8_t getDayFact(bool ExplicitF=false){
    DateArr src=getDateArr(ExplicitF);
    return src.data[MSM_IDAY];
  }

  uint8_t getDay(bool ExplicitF=false){
    DateArr src=getDateArr(ExplicitF);
    uint8_t rday=src.data[MSM_IDAY];
    DateArr::limitDay(rday,src.data[MSM_IMONTH],src.data[MSM_IYEAR]);
    return rday;
  }

  DateArr getDateArr(bool ExplicitF=false){
    if ((ExplicitF)||(epos<0))return cur;else return backup;
  }

  void setDateArr(DateArr newdate,bool alert=false){
    cur=newdate;
    setUpdated();
    if (alert) callUpdate();
  }

  void setDate(uint8_t nday,uint8_t nmonth,uint8_t nyear,bool alert=false){
    setDateArr(DateArr(nday,nmonth,nyear),alert);
  }
  
  bool incDate(uint8_t daystoadd,bool alert=false){
    TimeArr temp;
    bool foverflow=false;
    if (daystoadd){
      setUpdated();
      if (epos>=0) foverflow=backup.incDate(daystoadd); else {
        foverflow=cur.incDate(daystoadd);
        if (alert) callUpdate();
      }
    }
    return foverflow;
  }
  
  protected:
  DateArr cur,backup;
  int8_t epos;
};


#endif
