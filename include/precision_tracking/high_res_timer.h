/*
 * high_res_timer.h
 *
 *      Author: Alex Teichman
 *
 * A class for measuring how long some piece of code takes to run.
 *
 */

#ifndef __PRECISION_TRACKING__HIGH_RES_TIMER_H
#define __PRECISION_TRACKING__HIGH_RES_TIMER_H

#include <time.h>
#include <string>
#include <sstream>
#include <cstddef>
#include <iostream>
#include <cstdio>

namespace precision_tracking {

//! CLOCK_MONOTONIC_RAW will not be adjusted by NTP.
//! See man clock_gettime.
class HighResTimer {
public:
  std::string description_;

  HighResTimer(const std::string& description = "HighResTimer",
               const clockid_t& clock = CLOCK_PROCESS_CPUTIME_ID);
  void start();
  void stop();
  void reset(const std::string& description);
  void reset();
  double getMicroseconds() const;
  double getMilliseconds() const;
  double getSeconds() const;
  double getMinutes() const;
  double getHours() const;

  std::string report() const;
  std::string reportMicroseconds() const;
  std::string reportMilliseconds() const;
  std::string reportSeconds() const;
  std::string reportMinutes() const;
  std::string reportHours() const;
  
  void print() const {std::string msString = report(); printf("[TIMER] %s\n", msString.c_str());}
  void printSeconds() const {std::string msString = reportSeconds(); printf("[TIMER] %s\n", msString.c_str());}
  void printMilliseconds() const {std::string msString = reportMilliseconds(); printf("[TIMER] %s\n", msString.c_str());}
  void printMicroseconds() const {std::string msString = reportMicroseconds(); printf("[TIMER] %s\n", msString.c_str());}

private:
  double total_us_;
  timespec start_;
  timespec end_;
  clockid_t clock_;
};

class ScopedTimer
{
public:
  HighResTimer hrt_;
  ScopedTimer(const std::string& description = "ScopedTimer");
  ~ScopedTimer();
};

} // namespace precision_tracking

#endif // __PRECISION_TRACKING__HIGH_RES_TIMER_H
