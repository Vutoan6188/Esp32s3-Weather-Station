#ifndef VIETNAMESE_LUNAR_H
#define VIETNAMESE_LUNAR_H

#include <Arduino.h>

struct LunarDate {
  int day;
  int month;
  int year;
  bool leap;
};

// ---------------- Internal Functions ----------------

static long jdFromDate(int dd, int mm, int yy) {
  int a = (14 - mm) / 12;
  int y = yy + 4800 - a;
  int m = mm + 12 * a - 3;
  long jd = dd + (153 * m + 2) / 5 + y * 365 + y / 4 - y / 100 + y / 400 - 32045;
  return jd;
}

static int getNewMoonDay(int k, float timeZone) {
  float T = k / 1236.85f;
  float T2 = T * T;
  float T3 = T2 * T;
  float dr = PI / 180.0f;

  float Jd1 = 2415020.75933f + 29.53058868f * k
               + 0.0001178f * T2
               - 0.000000155f * T3;

  Jd1 += 0.00033f * sin((166.56f + 132.87f * T - 0.009173f * T2) * dr);

  float M = 359.2242f + 29.10535608f * k - 0.0000333f * T2 - 0.00000347f * T3;
  float Mpr = 306.0253f + 385.81691806f * k + 0.0107306f * T2 + 0.00001236f * T3;
  float F = 21.2964f + 390.67050646f * k - 0.0016528f * T2 - 0.00000239f * T3;

  float C1 = (0.1734f - 0.000393f * T) * sin(M * dr)
           + 0.0021f * sin(2 * M * dr)
           - 0.4068f * sin(Mpr * dr)
           + 0.0161f * sin(2 * Mpr * dr)
           - 0.0004f * sin(3 * Mpr * dr)
           + 0.0104f * sin(2 * F * dr)
           - 0.0051f * sin((M + Mpr) * dr)
           - 0.0074f * sin((M - Mpr) * dr)
           + 0.0004f * sin((2 * F + M) * dr)
           - 0.0004f * sin((2 * F - M) * dr)
           - 0.0006f * sin((2 * F + Mpr) * dr)
           + 0.0010f * sin((2 * F - Mpr) * dr)
           + 0.0005f * sin((2 * Mpr + M) * dr);

  float JdNew = Jd1 + C1;
  return (int)(JdNew + 0.5f + timeZone / 24.0f);
}

static int getSunLong(long jdn, float timeZone) {
  float T = (jdn - 2451545.5f - timeZone / 24.0f) / 36525.0f;
  float T2 = T * T;
  float dr = PI / 180.0f;

  float M = 357.52910f + 35999.05030f * T - 0.0001559f * T2 - 0.00000048f * T * T2;
  float L0 = 280.46645f + 36000.76983f * T + 0.0003032f * T2;
  float DL = (1.914600f - 0.004817f * T - 0.000014f * T2) * sin(M * dr)
           + (0.019993f - 0.000101f * T) * sin(2 * M * dr)
           + 0.000290f * sin(3 * M * dr);
  float L = L0 + DL;

  L *= dr;
  L -= 2 * PI * ((int)(L / (2 * PI)));

  return (int)(L / PI * 6.0f);
}

static long getLunarMonth11(int yy, float timeZone) {
  long off = jdFromDate(31, 12, yy) - 2415021;
  int k = (int)(off / 29.530588853f);
  long nm = getNewMoonDay(k, timeZone);
  long sunLong = getSunLong(nm, timeZone);

  if (sunLong >= 9) nm = getNewMoonDay(k - 1, timeZone);
  return nm;
}

static int getLeapMonthOffset(long a11, float timeZone) {
  int k = (int)((a11 - 2415021) / 29.530588853f);
  int last = 0;
  int i = 1;
  long arc = getSunLong(getNewMoonDay(k + i, timeZone), timeZone);

  do {
    last = arc;
    i++;
    arc = getSunLong(getNewMoonDay(k + i, timeZone), timeZone);
  } while (arc != last && i < 14);

  return i - 1;
}

// ---------------- Main Convert Function ----------------
static LunarDate convertSolar2Lunar(int dd, int mm, int yy, float timeZone = 7.0) {
  long dayNumber = jdFromDate(dd, mm, yy);
  long k = (long)((dayNumber - 2415021.076998695) / 29.530588853);

  long monthStart = getNewMoonDay(k + 1, timeZone);
  if (monthStart > dayNumber) monthStart = getNewMoonDay(k, timeZone);

  long a11 = getLunarMonth11(yy, timeZone);
  long b11 = getLunarMonth11(yy + 1, timeZone);

  int lunarYear;
  if (monthStart < a11) {
    a11 = getLunarMonth11(yy - 1, timeZone);
    b11 = getLunarMonth11(yy, timeZone);
    lunarYear = yy - 1;
  } else {
    lunarYear = yy;
  }

  int diff = (int)((monthStart - a11) / 29);
  int lunarMonth = diff + 11;
  bool leap = false;

  if (b11 - a11 > 365) {
    int leapMonth = getLeapMonthOffset(a11, timeZone);
    if (diff == leapMonth) leap = true;
    if (diff >= leapMonth) lunarMonth = diff + 10;
  }

  if (lunarMonth > 12) lunarMonth -= 12;
  if (lunarMonth >= 11 && diff < 4) lunarYear -= 1;

  int lunarDay = dayNumber - monthStart + 1;
  return { lunarDay, lunarMonth, lunarYear, leap };
}

/*static LunarDate convertSolar2Lunar(int dd, int mm, int yy, float timeZone = 7.0) {
  long dayNumber = jdFromDate(dd, mm, yy);
  long k = (long)((dayNumber - 2415021.076998695) / 29.530588853);

  long monthStart = getNewMoonDay(k + 1, timeZone);
  if (monthStart > dayNumber) monthStart = getNewMoonDay(k, timeZone);

  long a11 = getLunarMonth11(yy, timeZone);
  long b11 = getLunarMonth11(yy + 1, timeZone);

  int lunarYear = yy;
  int lunarDay = dayNumber - monthStart + 1;
  int diff = (int)((monthStart - a11) / 29);

  bool leap = false;
  int lunarMonth = diff + 11;

  if (b11 - a11 > 365) {
    int leapMonth = getLeapMonthOffset(a11, timeZone);
    if (diff == leapMonth) leap = true;
    if (diff >= leapMonth) lunarMonth = diff + 10;
  }

  if (lunarMonth > 12) {
    lunarMonth -= 12;
  }
  if (lunarMonth >= 11 && diff < 4) {
    lunarYear -= 1;
  }

  LunarDate ld = { lunarDay, lunarMonth, lunarYear, leap };
  return ld;
}*/

#endif