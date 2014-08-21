/*
 * fast_functions.h
 *
 *  Created on: Jan 4, 2014
 *      Author: davheld
 */

#ifndef FAST_FUNCTIONS_H_
#define FAST_FUNCTIONS_H_

#include <vector>

// Singleton Class.
class FastFunctions {
public:
  virtual ~FastFunctions();

  static FastFunctions& getInstance()
  {
      static FastFunctions instance;
      return instance;
  }

  double getFastLog(const double n) const;

private:

  FastFunctions();
  FastFunctions(FastFunctions const&);              // Don't Implement.
  void operator=(FastFunctions const&); // Don't implement

  std::vector<double> logs_;
  double log_resolution_;
  double max_log_;
};

#endif /* FAST_FUNCTIONS_H_ */
