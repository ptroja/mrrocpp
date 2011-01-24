/*
 * vs_logger.h
 *
 *  Created on: Jul 28, 2010
 *      Author: mboryn
 */

#ifndef VS_LOGGER_H_
#define VS_LOGGER_H_

#include <cstdio>
#include <string>

namespace mrrocpp {
namespace lib {
class Homog_matrix;
class configurator;
}

namespace ecp {

namespace servovision {

/** @addtogroup servovision
 *  @{
 */

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
