// Menu Structure Module main module
// Defenition of MSM classes

#ifndef __MSM_H_
#define __MSM_H_ 1

#include <inttypes.h>
#include "strservice.h"
#include "strloadARM.h"
#include "msmconfig.h"
#include "msm-ids.h"

// configuration constants
// #define MSM_FLARR_SIZE ((MSM_MENU_MAXLENGTH+7)>>3)
#define MSM_DFLARR_SIZE ((MSM_MENU_MAXLENGTH+7)>>3)

class MSMaList;

// interface of editable item contents
class IOpenItem{
  public:
  // start editing
  virtual void forceOpen()=0;
  // passkeys return flag if still open
  virtual bool passKeys(uint8_t key,uint8_t count)=0;
//  // dicard all editing changes and close editing
//  virtual void forceClose()=0;
//  // get flag if opened
//  virtual bool isOpened()=0;
  // get contents length
  virtual uint8_t getLength()=0;
  // get first and last positions of parameter edited (returns 0, length-1 for simple objects)
  virtual void getSelPos(uint8_t &firstp, uint8_t &lastp)=0;
  //convert to string
  virtual void toStr(char* str)=0;
};

// This class describes a menu item

class MSMItem{
public:
  MSMItem(const char* name,uint8_t newflags);
  MSMItem(const char* name,uint8_t newflags, void (*func) (void));
  MSMItem(const char* name,uint8_t newflags, IOpenItem * iptr);
  MSMItem(const char* name,uint8_t newflags, MSMaList * iptr);
#ifdef MSM_MSITEM_HAS_VIRTUAL_DESTRUCTOR
  virtual ~MSMItem();
#endif
  //for all items
  virtual void getName(char* str,uint8_t lLim);//lLim=MSM_MENU_STRLENGTH
  virtual void Enter();
  // for openables
  virtual bool passKeys(uint8_t key,uint8_t count); //returns true if still open
  virtual void getSelection(uint8_t &firstp, uint8_t &lastp,uint8_t lLim);//lLim=MSM_MENU_STRLENGTH
  //for submenu items
  void setList(MSMaList * newlist);
  MSMaList* getList();
  // flag set and check methods
  bool getUpdated();
  void setUpdated();
  void clearUpdated();
  bool ramName();
  uint8_t getMode();
  bool modeNormal();
  bool modeCloseLevel();
  bool modeCloseAll();
  bool modeSubMenu();
  bool modeOp();
protected:
  MSMItem(uint8_t newflags,const char* name,void * rptr);
  void loadName(char* str,uint8_t lLim);
  const char* _name;
  void* _rptr;
private:
  uint8_t _flags;
};

class MSMaList{
public:
  MSMaList(bool (*func)(uint8_t,uint8_t),MSMItem* title);
  //MSMaList(bool (*func)(uint8_t,uint8_t),MSMItem* title,MSMItem* itm0,MSMItem* itm1=0,MSMItem* itm2=0,MSMItem* itm3=0,MSMItem* itm4=0,MSMItem* itm5=0,MSMItem* itm6=0,MSMItem* itm7=0,MSMItem* itm8=0);
  MSMItem* getTitle();
  MSMItem* getItem(uint8_t n);
  uint8_t getNum();
  
  bool addItem(MSMItem * newitem,uint8_t n=0xff);
  bool delItem(uint8_t n);
  MSMItem* swapItem(MSMItem * newitem,uint8_t n);

  bool getUpdated(uint8_t n);
  void setUpdated(uint8_t n);
  void setUpdatedFrom(uint8_t n);
  void clearUpdated();
  inline bool docall(uint8_t num,uint8_t nstep);
  
protected:
  virtual MSMItem** getItemsArr()=0;
  virtual uint8_t* getListUpdateFlags()=0;
  virtual uint8_t getLength()=0;
  virtual uint8_t getFlagsLength()=0;

  MSMItem* _title;
  uint8_t _nitems;

  bool (*listcall)(uint8_t,uint8_t);
//  MSMItem* _items[MSM_MENU_MAXLENGTH];
//  uint8_t _luflags[MSM_FLARR_SIZE]; // list update flags
};

class MSMDisplay{
public:
  void reset(MSMaList* rootlist, uint8_t height=0);
  MSMDisplay(uint8_t height);
  void ProcessKeys(uint8_t key,uint8_t count);

  bool isopen();
  bool itemopen();
  uint8_t getHeight();
  uint8_t getPos();
  uint8_t getTop();
  MSMaList* getList();

  uint8_t getLevel();  
  uint8_t getPosAtLevel(uint8_t nlevel);
  
  bool getUpdated(uint8_t n=0xff);
  bool getUpdatedAny();
  void setUpdated(uint8_t n=0xff);
  void setUpdatedFrom(uint8_t n=0xff);
  void clearUpdated();
  
protected:
//  uint8_t _width;
  void listEnter();
  void listDown();
  void listUp();
  void listClose();
  bool listOpen(MSMaList* newlist);

  uint8_t _height;
  uint8_t _level;
  uint8_t _pos[MSM_MENU_MAXDEPTH];
  uint8_t _top[MSM_MENU_MAXDEPTH];
  MSMaList* _list[MSM_MENU_MAXDEPTH];
  uint8_t _duflags[MSM_DFLARR_SIZE]; // display update flags
  bool ifiopen;
};

// ----------------------
// MSMItems methods
// ----------------------

//constructor with no pointer (appliable for all modes,reasonable for CloseLevel and CloseAll modes)
inline MSMItem::MSMItem(const char* name,uint8_t newflags): _name(name),_rptr(0),_flags(newflags){}

// constructor with function call, appliable for Normal CloseLevel and CloseAll modes
inline MSMItem::MSMItem(const char* name,uint8_t newflags, void (*func)(void)): _name(name),_rptr((void*)func), _flags(newflags){
  // Here we check if the combination of flags is correct for a procedure call item, and correct flags if the set is incorrect
  if (!(modeNormal()||modeCloseLevel()||modeCloseAll())) _flags=_flags&MSM_FL_MM;
}

// constructor for item with external openable object
inline MSMItem::MSMItem(const char* name,uint8_t newflags, IOpenItem * iptr): _name(name),_rptr((void*) iptr),_flags((newflags&MSM_FL_MM)+MSM_MODE_OP){}

// constructor for item with submenu if no autolink** used
// **constructor of MSMaList automatically calls setList() method of its title
// ** This methof links _rptr of item to the list if the item is of SUBMENU mode 
inline MSMItem::MSMItem(const char* name,uint8_t newflags, MSMaList * iptr): _name(name),_rptr((void*)iptr),_flags((newflags&MSM_FL_MM)+MSM_MODE_SUBMENU){}

//private constructor to be used with child openable objects
inline MSMItem::MSMItem(uint8_t newflags,const char* name,void * rptr): _name(name),_rptr(rptr), _flags((newflags&MSM_FL_MM)+MSM_MODE_OP){}

#ifdef MSM_MSITEM_HAS_VIRTUAL_DESTRUCTOR
inline  MSMItem::~MSMItem(){};
#endif

inline void MSMItem::loadName(char* str,uint8_t lLim){
  if (ramName()){
    strCpyL(str,_name,lLim);
  }else{
    pgmLoadStr(str,_name,lLim);
  }
  clearUpdated();
}

inline void MSMItem::getName(char* str,uint8_t lLim){
  loadName(str,lLim);
  if((modeOp())&&(_rptr!=0)){
    // get data pRef op
    uint8_t L=((IOpenItem*)_rptr)->getLength();
    strFix(str,lLim);
    if (lLim>=L){
       ((IOpenItem*)_rptr)->toStr(&str[lLim-L]);//
    }
  }
}

inline void MSMItem::Enter(){
  if ((modeNormal()||modeCloseLevel()||modeCloseAll())&&(_rptr!=0)){
    void (*func)(void)= (void(*)(void))_rptr;
    (*func)();
  }else if((modeOp())&&(_rptr!=0)){
    ((IOpenItem*)_rptr)->forceOpen();
    setUpdated();
  }
}

inline bool MSMItem::passKeys(uint8_t key,uint8_t count){
  if((modeOp())&&(_rptr!=0)){
    setUpdated();  
    return ((IOpenItem*)_rptr)->passKeys(key,count);
  }else return false;
}

inline void MSMItem::getSelection(uint8_t &firstp, uint8_t &lastp,uint8_t lLim){
  if((modeOp())&&(_rptr!=0)){
    uint8_t L=((IOpenItem*)_rptr)->getLength();
    if (lLim>=L){
      ((IOpenItem*)_rptr)->getSelPos(firstp,lastp);
      L=lLim-L;
      firstp+=L;
      lastp+=L;
      if (firstp>=lLim)firstp=lLim-1;
      if (lastp>=lLim)lastp=lLim-1;
    }else {
      firstp=1; // if we have a problem with length  
      lastp=1;  // only first character will be selected
    }
  }else{
    firstp=1;      // if we have a getselection call in wrong mode or no IOpenItem assotiated
    lastp=lLim-2;  // we have all chars but last highlighted
  } 
}

inline void MSMItem::setList(MSMaList * newlist){
  if (modeSubMenu()) _rptr=newlist;
}

inline MSMaList * MSMItem::getList(){
  if (modeSubMenu()) return (MSMaList*)_rptr; else return 0;
}

inline bool MSMItem::getUpdated(){
  return (_flags&(MSM_FL_UPDATED|MSM_FL_LIVEUPDATE));
}

inline void MSMItem::setUpdated(){
  _flags|=MSM_FL_UPDATED;
}

inline void MSMItem::clearUpdated(){
  _flags&=~(MSM_FL_UPDATED);
}

inline bool MSMItem::ramName(){
  return _flags&MSM_FL_RAMNAME;
}

inline uint8_t MSMItem::getMode(){
  return _flags & MSM_MODE_MM;
}

inline bool MSMItem::modeNormal(){
  return (_flags& MSM_MODE_MM) == MSM_MODE_NORMAL;
}

inline bool MSMItem::modeCloseLevel(){
  return (_flags& MSM_MODE_MM) == MSM_MODE_CLOSELEVEL;
}

inline bool MSMItem::modeCloseAll(){
  return (_flags & MSM_MODE_MM) == MSM_MODE_CLOSEALL;
}

inline bool MSMItem::modeSubMenu(){
  return (_flags & MSM_MODE_MM) == MSM_MODE_SUBMENU;
}

inline bool MSMItem::modeOp(){
  return (_flags & MSM_MODE_MM) == MSM_MODE_OP;
}

// ----------------------
// MSMaList methods
// ----------------------

inline MSMaList::MSMaList(bool (*func)(uint8_t,uint8_t),MSMItem* title):_title(title),_nitems(0),listcall(func){
  title->setList(this); // link title item to its submenu
}

inline MSMItem* MSMaList::getTitle(){
  return _title;
}
inline MSMItem* MSMaList::getItem(uint8_t n){
  MSMItem** _items=getItemsArr();
  return _items[n];  
}
inline uint8_t MSMaList::getNum(){
  return _nitems;  
}

inline bool MSMaList::addItem(MSMItem * newitem,uint8_t n){
  MSMItem** _items=getItemsArr();
  if(_nitems==getLength()) return false;
  if(n==0xff) n=_nitems;
  if (_nitems) for (uint8_t i=_nitems-1;i>=n;--i) _items[i+1]=_items[i];
  _items[n]= newitem;
  _nitems++;
  setUpdatedFrom(n);
  return true;
}

inline bool MSMaList::delItem(uint8_t n){
  MSMItem** _items=getItemsArr();
  if(n+1<_nitems){
    for(uint8_t i=n;i+1<_nitems;++i) _items[i]=_items[i+1];
    _nitems--;
    _items[_nitems]=0;
    setUpdatedFrom(n);
    return true;
  }
  return false;
}

inline MSMItem* MSMaList::swapItem(MSMItem * newitem,uint8_t n){
  MSMItem** _items=getItemsArr();
  MSMItem* tmp=_items[n];
  _items[n]=newitem;
  setUpdated(n);
  return tmp;
}

inline bool MSMaList::getUpdated(uint8_t n){
  uint8_t* _luflags=getListUpdateFlags();
  return _luflags[(n>>3)]&(1<<(n&0x07));
}

inline void MSMaList::setUpdated(uint8_t n){
//  if (n>=MSM_MENU_MAXLENGTH) return;
  uint8_t* _luflags=getListUpdateFlags();
  _luflags[(n>>3)]|=(1<<(n&0x07));
}

inline void MSMaList::setUpdatedFrom(uint8_t n){
//  if (n>=MSM_MENU_MAXLENGTH) return;
  uint8_t* _luflags=getListUpdateFlags();
  _luflags[(n>>3)]|=(0xff<<(n&0x07));
  for (uint8_t i=(n>>3)+1;i<getFlagsLength();++i)_luflags[i]=0xff;
}

inline void MSMaList::clearUpdated(){
  uint8_t* _luflags=getListUpdateFlags();
  for (uint8_t i=0;i<getFlagsLength();++i) _luflags[i]=0;
}

inline bool MSMaList::docall(uint8_t num,uint8_t nstep){
  if (listcall){return (*listcall)(num,nstep);}else return true;
}

// ----------------------
// MSMDisplay methods
// ----------------------

inline void MSMDisplay::reset(MSMaList* rootlist,uint8_t height){
    uint8_t i;
    if (height) _height=height;
    for (i=0;i<MSM_MENU_MAXDEPTH;++i){
      _pos[i]=0;
      _top[i]=0;
      _list[i]=0;
    }
    ifiopen=false;    
    _level=0;
    _list[0]=rootlist;
    setUpdatedFrom();
  }
  
inline MSMDisplay::MSMDisplay(uint8_t height):_height(height){
    reset(0);
  }
  
inline bool MSMDisplay::isopen(){
    return _list[0];
  }
inline bool MSMDisplay::itemopen(){
  return ifiopen;
}

inline uint8_t MSMDisplay::getHeight(){
  return _height;
}

inline void MSMDisplay::ProcessKeys(uint8_t key,uint8_t count){
  if ((!isopen())||(key==0))return;
  if((key==MSM_KEY_ESC)&&(count>MSM_LIM_COUNT)){reset(0);return;}  //Emergency break
  if (ifiopen){
    //if current item open 
    MSMItem * citem=_list[_level]->getItem(_pos[_level]);
    if (citem==0){ifiopen=false; return;}
    if((key==MSM_KEY_ESC)&&(count>MSM_BRK_COUNT)){
      ifiopen=false;
      return;
    }else{
      ifiopen=citem->passKeys(key,count);
      return;
    }
  }
  //if current item not open move over items
  if(key==MSM_KEY_UP) listUp();else
  if(key==MSM_KEY_DOWN) listDown();else  
  if((key==MSM_KEY_ESC)&&(count==0)) listClose();else
  if((key==MSM_KEY_ENTER)&&(count==0)) listEnter();
}

inline void MSMDisplay::listEnter(){
  if (_list[_level]==0)return;
  bool callf=_list[_level]->docall(_pos[_level],LISTCALSE_BEFORE);//pre item call, gets flag to perform item enter
  MSMItem * citem=_list[_level]->getItem(_pos[_level]);
  if ((callf)&&(citem)){
    citem->Enter();
    if (citem->modeOp()) ifiopen=true;
  }
  callf=_list[_level]->docall(_pos[_level],LISTCALSE_AFTER);//post item call, gets result flag
  if ((callf)&&(citem)){  
    if (citem->modeCloseLevel()) listClose(); else
    if (citem->modeCloseAll()) reset(0); else
    if (citem->modeSubMenu()) listOpen(citem->getList());
  }
}

inline void MSMDisplay::listUp(){
  if (_pos[_level]>0){
    setUpdated(_pos[_level]);
    _pos[_level]--;
    setUpdated(_pos[_level]);
    if(_top[_level]>_pos[_level]){
      _top[_level]=_pos[_level];
      setUpdatedFrom(_pos[_level]);
    }
  }
}

inline void MSMDisplay::listDown(){
  // get number of items
  uint8_t n = _list[_level]->getNum();
  if ((n>1)&&(_pos[_level]<n-1)){
    setUpdated(_pos[_level]);
    _pos[_level]++;
    setUpdated(_pos[_level]);
    if((_pos[_level]-_top[_level])>=_height){
      _top[_level]=_pos[_level]+1-_height;
      setUpdatedFrom(_top[_level]);      
    }
  }
}

inline void MSMDisplay::listClose(){
  _list[_level]=0;
  if(_level) _level--;
  setUpdatedFrom();
}

inline bool MSMDisplay::listOpen(MSMaList* newlist){
  if (newlist==0) return false;
  if (_level>=MSM_MENU_MAXDEPTH-1) return false;
  _level++;
  _list[_level]=newlist;
  _pos[_level]=0;
  _top[_level]=0;
  setUpdatedFrom();
  return true;
}

inline uint8_t MSMDisplay::getPos(){
  return _pos[_level];
}

inline uint8_t MSMDisplay::getTop(){
  return _top[_level];
}

inline MSMaList* MSMDisplay::getList(){
  return _list[_level];
}

inline uint8_t MSMDisplay::getLevel(){
  return _level;
}  

inline uint8_t MSMDisplay::getPosAtLevel(uint8_t nlevel){
  return _pos[nlevel];
}  

inline bool MSMDisplay::getUpdated(uint8_t n){
  n+=1;
  return _duflags[(n>>3)]&(1<<(n&0x07));
}

inline bool MSMDisplay::getUpdatedAny(){
  for(uint8_t i=0;i<MSM_DFLARR_SIZE;++i){
    if (_duflags[i])return true;
  }
  return false;
}

inline void MSMDisplay::setUpdated(uint8_t n){
  n+=1;
//  if (n>MSM_MENU_MAXLENGTH) return;
  _duflags[(n>>3)]|=(1<<(n&0x07));
}

inline void MSMDisplay::setUpdatedFrom(uint8_t n){
  n+=1;
//  if (n>MSM_MENU_MAXLENGTH) return;
  _duflags[(n>>3)]|=(0xff<<(n&0x07));
  for (uint8_t i=(n>>3)+1;i<MSM_DFLARR_SIZE;++i)_duflags[i]=0xff;
}

inline void MSMDisplay::clearUpdated(){
  for (uint8_t i=0;i<MSM_DFLARR_SIZE;++i) _duflags[i]=0;
}

template<uint8_t listLength> 
class MSMtList:public MSMaList {
  // This is important limitation but requires c++ 11 
  // static_assert( (listLength>MSM_MENU_MAXLENGTH ),"MSMtList template parameter listLength should not exceed MSM_MENU_MAXLENGTH");
public:
  // we use such a clumsy constuctor here to be shure that all itmX parameters are MSMItem* 
  MSMtList(bool (*func)(uint8_t,uint8_t),MSMItem* title,MSMItem* itm0,MSMItem* itm1=0,MSMItem* itm2=0,MSMItem* itm3=0,MSMItem* itm4=0,MSMItem* itm5=0,MSMItem* itm6=0,MSMItem* itm7=0,MSMItem* itm8=0,MSMItem* itm9=0,MSMItem* itm10=0,MSMItem* itm11=0,MSMItem* itm12=0,MSMItem* itm13=0,MSMItem* itm14=0,MSMItem* itm15=0):MSMaList(func,title){
//  title->setList(this); // link title item to its submenu
    MSMItem** _items=getItemsArr();
    for (uint8_t i=0;i<getLength();++i) _items[i]=0;
    _items[0]=itm0;_nitems=1;
    for (uint8_t i=1;i<listLength;++i)_items[i]=0;
    if((itm1!=0)&&(listLength>_nitems)) {_items[1]=itm1;_nitems=2;}else return;
    if((itm2!=0)&&(listLength>_nitems)) {_items[2]=itm2;_nitems=3;}else return;
    if((itm3!=0)&&(listLength>_nitems)) {_items[3]=itm3;_nitems=4;}else return;
    if((itm4!=0)&&(listLength>_nitems)) {_items[4]=itm4;_nitems=5;}else return;
    if((itm5!=0)&&(listLength>_nitems)) {_items[5]=itm5;_nitems=6;}else return;
    if((itm6!=0)&&(listLength>_nitems)) {_items[6]=itm6;_nitems=7;}else return;
    if((itm7!=0)&&(listLength>_nitems)) {_items[7]=itm7;_nitems=8;}else return;
    if((itm8!=0)&&(listLength>_nitems)) {_items[8]=itm8;_nitems=9;}else return;
    if((itm9!=0)&&(listLength>_nitems)) {_items[9]=itm9;_nitems=10;}else return;
    if((itm10!=0)&&(listLength>_nitems)) {_items[10]=itm10;_nitems=11;}else return;
    if((itm11!=0)&&(listLength>_nitems)) {_items[11]=itm11;_nitems=12;}else return;
    if((itm12!=0)&&(listLength>_nitems)) {_items[12]=itm12;_nitems=13;}else return;
    if((itm13!=0)&&(listLength>_nitems)) {_items[13]=itm13;_nitems=14;}else return;
    if((itm14!=0)&&(listLength>_nitems)) {_items[14]=itm14;_nitems=15;}else return;
    if((itm15!=0)&&(listLength>_nitems)) {_items[15]=itm15;_nitems=16;}else return;
  }

protected:  
  virtual MSMItem** getItemsArr(){
    return &_items[0];  //MSMItem** _items=getItemsArr();
  }
  virtual uint8_t* getListUpdateFlags(){
    return &_luflags[0]; //uint8_t* _luflags=getListUpdateFlags();
  }
  virtual uint8_t getLength(){
    return listLength;
  }

  virtual uint8_t getFlagsLength(){
    return ((listLength+7)>>3);
  }

  MSMItem* _items[listLength];
  uint8_t _luflags[((listLength+7)>>3)]; // list update flags
};

#endif
