
#ifndef VIETNAMESE_LUNAR_H
#define VIETNAMESE_LUNAR_H

#include <Arduino.h>
#include <math.h>

struct LunarDate {
  int day;
  int month;
  int year;
  bool leap;
};

static constexpr double VL_PI = 3.14159265358979323846;
static constexpr double VL_TZ = 7.0;

// Julian Day from Gregorian date
static long jdFromDate(int dd, int mm, int yy) {
  int a = (14 - mm) / 12;
  int y = yy + 4800 - a;
  int m = mm + 12 * a - 3;

  return dd + (153 * m + 2) / 5
         + 365L * y + y / 4 - y / 100 + y / 400 - 32045;
}

// New moon day in local time
static long getNewMoonDay(int k, double timeZone) {
  double T = k / 1236.85;
  double T2 = T * T;
  double T3 = T2 * T;
  double dr = VL_PI / 180.0;

  double Jd1 = 2415020.75933
               + 29.53058868 * k
               + 0.0001178 * T2
               - 0.000000155 * T3;

  Jd1 += 0.00033 * sin((166.56 + 132.87 * T - 0.009173 * T2) * dr);

  double M = 359.2242 + 29.10535608 * k
             - 0.0000333 * T2 - 0.00000347 * T3;

  double Mpr = 306.0253 + 385.81691806 * k
               + 0.0107306 * T2 + 0.00001236 * T3;

  double F = 21.2964 + 390.67050646 * k
             - 0.0016528 * T2 - 0.00000239 * T3;

  double C1 = (0.1734 - 0.000393 * T) * sin(M * dr)
              + 0.0021 * sin(2.0 * M * dr)
              - 0.4068 * sin(Mpr * dr)
              + 0.0161 * sin(2.0 * Mpr * dr)
              - 0.0004 * sin(3.0 * Mpr * dr)
              + 0.0104 * sin(2.0 * F * dr)
              - 0.0051 * sin((M + Mpr) * dr)
              - 0.0074 * sin((M - Mpr) * dr)
              + 0.0004 * sin((2.0 * F + M) * dr)
              - 0.0004 * sin((2.0 * F - M) * dr)
              - 0.0006 * sin((2.0 * F + Mpr) * dr)
              + 0.0010 * sin((2.0 * F - Mpr) * dr)
              + 0.0005 * sin((2.0 * Mpr + M) * dr);

  double JdNew = Jd1 + C1;

  double deltaT;
  if (T < -11.0) {
    deltaT = 0.001
             + 0.000839 * T
             + 0.0002261 * T2
             - 0.00000845 * T3
             - 0.000000081 * T * T3;
  } else {
    deltaT = -0.000278
             + 0.000265 * T
             + 0.000262 * T2;
  }

  return (long)floor(JdNew + 0.5 + timeZone / 24.0
                     - deltaT / 86400.0);
}

// Sun longitude sector: 0..11
static int getSunLong(long jdn, double timeZone) {
  double T = (jdn - 2451545.5 - timeZone / 24.0)
             / 36525.0;
  double T2 = T * T;
  double dr = VL_PI / 180.0;

  double M = 357.52910 + 35999.05030 * T
             - 0.0001559 * T2
             - 0.00000048 * T * T2;

  double L0 = 280.46645 + 36000.76983 * T
              + 0.0003032 * T2;

  double DL = (1.914600 - 0.004817 * T
               - 0.000014 * T2)
                * sin(M * dr)
              + (0.019993 - 0.000101 * T)
                  * sin(2.0 * M * dr)
              + 0.000290 * sin(3.0 * M * dr);

  double L = (L0 + DL) * dr;
  L -= 2.0 * VL_PI * floor(L / (2.0 * VL_PI));

  return (int)floor(L / VL_PI * 6.0);
}

// Lunar month 11
static long getLunarMonth11(int yy, double timeZone) {
  long off = jdFromDate(31, 12, yy) - 2415021L;
  int k = (int)floor(off / 29.530588853);

  long nm = getNewMoonDay(k, timeZone);
  int sunLong = getSunLong(nm, timeZone);

  if (sunLong >= 9) {
    nm = getNewMoonDay(k - 1, timeZone);
  }

  return nm;
}

// Leap month offset
static int getLeapMonthOffset(long a11, double timeZone) {
  int k = (int)floor(
    (a11 - 2415021.076998695) / 29.530588853 + 0.5);

  int last = 0;
  int i = 1;

  int arc = getSunLong(
    getNewMoonDay(k + i, timeZone), timeZone);

  do {
    last = arc;
    i++;
    arc = getSunLong(
      getNewMoonDay(k + i, timeZone), timeZone);
  } while (arc != last && i < 14);

  return i - 1;
}

// Solar date -> Vietnamese lunar date
static LunarDate convertSolar2Lunar(
  int dd, int mm, int yy, double timeZone = VL_TZ) {

  long dayNumber = jdFromDate(dd, mm, yy);

  int k = (int)floor(
    (dayNumber - 2415021.076998695) / 29.530588853);

  long monthStart = getNewMoonDay(k + 1, timeZone);
  if (monthStart > dayNumber) {
    monthStart = getNewMoonDay(k, timeZone);
  }

  long a11 = getLunarMonth11(yy, timeZone);
  long b11 = a11;
  int lunarYear;

  if (a11 >= monthStart) {
    lunarYear = yy;
    a11 = getLunarMonth11(yy - 1, timeZone);
  } else {
    lunarYear = yy + 1;
    b11 = getLunarMonth11(yy + 1, timeZone);
  }

  int lunarDay = (int)(dayNumber - monthStart + 1);
  int diff = (int)floor((monthStart - a11) / 29.0);

  bool leap = false;
  int lunarMonth = diff + 11;

  if (b11 - a11 > 365) {
    int leapMonth = getLeapMonthOffset(a11, timeZone);

    if (diff >= leapMonth) {
      lunarMonth = diff + 10;
      if (diff == leapMonth) {
        leap = true;
      }
    }
  }

  if (lunarMonth > 12) {
    lunarMonth -= 12;
  }

  if (lunarMonth >= 11 && diff < 4) {
    lunarYear--;
  }

  return { lunarDay, lunarMonth, lunarYear, leap };
}

#endif
