/*
 * Timer.h
 *
 *  Created on: 07-12-2015
 *      Author: felipe-narvaez
 */

#ifndef UTILS_TIMER_H_
#define UTILS_TIMER_H_
#include <chrono>
#include <iostream>


class Timer {
public:
	Timer();
	std::string report();
	void reset();
	double elapsed();
	virtual ~Timer();


private:
	typedef std::chrono::high_resolution_clock clock_;
	typedef std::chrono::duration<double, std::ratio<1> > second_;
	std::chrono::time_point<clock_> beg_;
};

#endif /* UTILS_TIMER_H_ */
