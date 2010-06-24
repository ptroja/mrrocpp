/*
 * TimeUtil.cpp
 *
 *  Created on: Mar 11, 2010
 *      Author: docent
 */

#include "TimeUtil.h"

#include <sys/syspage.h>
#include <sys/neutrino.h>

#include <fstream>

namespace mrrocpp {
namespace edp {
namespace sensor {

static uint64_t CPU_CLOCK = SYSPAGE_ENTRY(qtime)->cycles_per_sec;

TimeUtil::TimeUtil() {
	reset();
}

TimeUtil::~TimeUtil() {
	// TODO Auto-generated destructor stub
}

unsigned int TimeUtil::cyclesToUs(uint64_t cycles) {
	return static_cast<unsigned int> (cycles / (CPU_CLOCK / 1000000.0));
}

uint64_t TimeUtil::usToCycles(unsigned int us) {
	//printf("CPU_CLOCK = %llu\n", CPU_CLOCK);
	return us * (CPU_CLOCK / 1000000.0);
}

void TimeUtil::reset() {
	iter = 0;
	startCycle = 0;
	delayCycle = 0;
}

void TimeUtil::probeDelayCycle() {
	delayCycle = ClockCycles();
}

void TimeUtil::startMeasurement() {
	startCycle = ClockCycles();
}

void TimeUtil::stopMeasurement() {
	measurements[iter] = ClockCycles() - startCycle;
	++iter;
	if (iter % 1000 == 0) {
		unsigned int timeElapsedUs = cyclesToUs(ClockCycles() - mpsFirstCycle);
		//printf("Time elapsed us = %u\n", timeElapsedUs);
		mps_ = 1000000000ull / timeElapsedUs;
		//printf("base/mps = %u\n", mps_);
		mpsFirstCycle = ClockCycles();

	}
}

void TimeUtil::wait(unsigned int desiredPeriodUs) {
	int64_t cyclesToWait = delayCycle + usToCycles(desiredPeriodUs) - ClockCycles();
	if (cyclesToWait > 0) {
		unsigned int usToWait = cyclesToUs(cyclesToWait);
		usleep(usToWait);
	}
}

unsigned int TimeUtil::mps() {
	return mps_;
}

void TimeUtil::dump(const char* fileName) {
	std::ofstream dump(fileName);
		for (int i = 0; i < iter; ++i) {
			dump << (measurements[i] / (CPU_CLOCK / 1000000.0)) << '\n';
		}
	dump.close();
}

}
}
}
