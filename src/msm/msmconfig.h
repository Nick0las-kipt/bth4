#ifndef msmconfig_h
#define msmconfig_h

//------------------------------------------
// MSM configuration constants and functions
// -----------------------------------------

// configuration constants
// length and depth

// Maximum length of items array
// Maximum number of items = msm_menu_maxLength +1
#ifndef MSM_MENU_MAXLENGTH
#define MSM_MENU_MAXLENGTH    16
#endif

// Maximum depth of menu structure
// This patameter controls size of arrays in msmdistlay objects
#ifndef MSM_MENU_MAXDEPTH
#define MSM_MENU_MAXDEPTH      4
#endif

// Maximum length of msmstr ()
#ifndef MSM_MENU_STRLENGTH
#define MSM_MENU_STRLENGTH    23
#endif

typedef const char MSMStr[MSM_MENU_STRLENGTH+1];

// date and time display parameters
#define MSM_DATE_SEPARATOR '/';
#define MSM_TIME_SEPARATOR ':';
// indexes for day, month and year in timarr struct for date items
// for european format DD/MM/YY uncomment this
#define MSM_IDAY 0
#define MSM_IMONTH 1
#define MSM_IYEAR 2

// for american format MM/DD/YY uncomment this
// #define MSM_IDAY 1
// #define MSM_IMONTH 0
// #define MSM_IYEAR 2

// msmitem class has no virtual destructor by default for code size reduction
// uncomment this if you need msmitem destructor
// #define MSM_MSITEM_HAS_VIRTUAL_DESTRUCTOR

// count converter for fast increment or decrement integer item contents
template <typename Tparam>
void convertCount(uint8_t count,Tparam &delta){
    delta=1;
    if (count>0){
      delta=count-1;
      if ((sizeof(delta)>1)&&(count>4)) delta*=(delta-2);
      if ((sizeof(delta)>2)&&(count>5)) delta*=(Tparam)(count-5);
    }
}

#endif
