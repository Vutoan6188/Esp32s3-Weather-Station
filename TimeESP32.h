#pragma once

#include <time.h>

inline time_t getLocalTimeSafe() {

  time_t nowTime;
  time(&nowTime);

  return nowTime;
}

inline int getHour(time_t t) {

  struct tm tm;
  localtime_r(&t, &tm);

  return tm.tm_hour;
}

inline int getMinute(time_t t) {

  struct tm tm;
  localtime_r(&t, &tm);

  return tm.tm_min;
}

inline int getSecond(time_t t) {

  struct tm tm;
  localtime_r(&t, &tm);

  return tm.tm_sec;
}

inline int getDay(time_t t) {

  struct tm tm;
  localtime_r(&t, &tm);

  return tm.tm_mday;
}

inline int getMonth(time_t t) {

  struct tm tm;
  localtime_r(&t, &tm);

  return tm.tm_mon + 1;
}

inline int getYear(time_t t) {

  struct tm tm;
  localtime_r(&t, &tm);

  return tm.tm_year + 1900;
}

inline const char* dayStrESP32(time_t t) {

  static const char* days[] = {
    "Sunday",
    "Monday",
    "Tuesday",
    "Wednesday",
    "Thursday",
    "Friday",
    "Saturday"
  };

  struct tm tm;
  localtime_r(&t, &tm);

  return days[tm.tm_wday];
}