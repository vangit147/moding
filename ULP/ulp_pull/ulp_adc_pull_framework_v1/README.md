时间变量结构体成员变量
struct tm
{
int tm_sec;
int tm_min;
int tm_hour;
int tm_mday;
int tm_mon;
int tm_year;
int tm_wday;
int tm_yday;
int tm_isdst;
#ifdef **TM_GMTOFF
long **TM_GMTOFF;
#endif
#ifdef **TM_ZONE
const char \***TM_ZONE;
#endif
};
