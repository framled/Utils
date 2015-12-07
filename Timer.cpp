/*
 * Timer.cpp
 *
 *  Created on: 07-12-2015
 *      Author: felipe-narvaez
 */

#include "Timer.h"


Timer::~Timer() {
	// TODO Auto-generated destructor stub
}

Timer::Timer() : beg_(clock_::now()) {}

void Timer::reset() {
	beg_ = clock_::now();
}

double Timer::elapsed() {
	return std::chrono::duration_cast<second_> (clock_::now() - beg_).count();
}

std::string Timer::report(){
	return std::to_string(elapsed());
}
