/*
 * TimeUtil.h
 *
 *  Created on: Mar 11, 2010
 *      Author: docent
 */

#ifndef TIMEUTIL_H_
#define TIMEUTIL_H_

#include <boost/cstdint.hpp>

namespace mrrocpp {
namespace edp {
namespace sensor {


class TimeUtil {
	static const std::size_t MAX_MEASUREMENTS = 60 * 1000 * 10;

public:
	TimeUtil();
	virtual ~TimeUtil();
	void reset();
	void startMeasurement();
	void stopMeasurement();
	void probeDelayCycle();
	static unsigned int cyclesToUs(uint64_t cycles);
	static uint64_t usToCycles(unsigned int us);
	void wait(unsigned int desiredPeriodUs);
	unsigned int mps();
	void dump(const char* fileName);

private:
	uint64_t iter;
	uint64_t startCycle;
	uint64_t mpsFirstCycle;
	uint64_t delayCycle;
	unsigned int mps_;
	uint64_t measurements[MAX_MEASUREMENTS];
};

}
}
}

#endif /* TIMEUTIL_H_ */
