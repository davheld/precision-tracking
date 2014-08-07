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

  // Cache exp.
  exp_resolution_ = 0.01;
  max_exp_ = -5.0;
  const int exp_max_vals =  -max_exp_ / exp_resolution_;
  exps_.resize(exp_max_vals + 1);
  for (int i = 0; i <= exp_max_vals; ++i) {
    exps_[i] = exp(-i * exp_resolution_);
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

double FastFunctions::getFastExp(const double n) const {
  if (n >= max_exp_ && n <= 0) {
    const int exp_index = -n / exp_resolution_;
    //printf("Finding exp of %lf - index: %lf\n", n, exps_[exp_index]);
    return exps_[exp_index];
  } else if (n < max_exp_){
    //printf("Finding exp of %lf - 0\n", n);
    return 0;
  } else {
    //printf("Finding exp of %lf - slow\n", n);
    return exp(n);
  }
}
