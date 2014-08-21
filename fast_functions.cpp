/*
 * fast_functions.cpp
 *
 *  Created on: Jan 4, 2014
 *      Author: davheld
 */

#include "fast_functions.h"
#include <cmath>
#include <cstdio>

FastFunctions::FastFunctions() {
  // Cache logs.
  log_resolution_ = 0.01;
  max_log_ = 5.0;
  const int log_max_vals =  max_log_ / log_resolution_;
  logs_.resize(log_max_vals + 1);
  for (int i = 0; i <= log_max_vals; ++i) {
    logs_[i] = log(i * log_resolution_);
  }
}

FastFunctions::~FastFunctions() {
  // TODO Auto-generated destructor stub
}

double FastFunctions::getFastLog(const double n) const {
  if (n <= max_log_ && n > log_resolution_) {
    const int log_index = n / log_resolution_;
    return logs_[log_index];
  } else {
    return log(n);
  }
}
