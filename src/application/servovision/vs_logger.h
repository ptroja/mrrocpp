/*
 * vs_logger.h
 *
 *  Created on: Jul 28, 2010
 *      Author: mboryn
 */

#ifndef VS_LOGGER_H_
#define VS_LOGGER_H_

#include <cstdio>
#include <ctime>
#include <string>

#include <boost/thread/thread.hpp>
#include <boost/utility.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition_variable.hpp>
#include <boost/circular_buffer.hpp>

#include "base/lib/mrmath/mrmath.h"
#include "base/lib/configurator.h"

namespace mrrocpp {

namespace ecp {

namespace servovision {

/** @addtogroup servovision
 *  @{
 */

/**
 *
 */
struct data{
	struct timespec measure_time;
	lib::Homog_matrix error;
};

/**
 *
 */
class vs_logger
{
public:
	vs_logger(lib::configurator &config);
	virtual ~vs_logger();

	void start();
	void stop();

	void log(const lib::Homog_matrix &O_T_E, const lib::Homog_matrix &O_T_G, const lib::Homog_matrix &C_T_G, const lib::Homog_matrix &error);

//	void operator()();
protected:

private:
	FILE *fp;
	lib::configurator &config;
	std::string reader_meassures_dir;
	int counter;
	static const int MAX_ENTRIES = 1000;
};

/** @} */


}//namespace servovision

}//namespace ecp

}//namespace mrrocpp

#endif /* VS_LOGGER_H_ */
