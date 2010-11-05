/*
 * ecp_g_discode_sensor_test.h
 *
 *  Created on: Nov 4, 2010
 *      Author: mboryn
 */

#ifndef ECP_G_DISCODE_SENSOR_TEST_H_
#define ECP_G_DISCODE_SENSOR_TEST_H_

#include "base/ecp/ecp_generator.h"

#include "sensor/discode/discode_sensor.h"

namespace mrrocpp {

namespace ecp {

namespace common {

namespace generator {

class ecp_g_discode_sensor_test : public mrrocpp::ecp::common::generator::generator
{
public:
	ecp_g_discode_sensor_test(mrrocpp::ecp::common::task::task & ecp_task, mrrocpp::ecp_mp::sensor::discode::discode_sensor &ds);
	virtual ~ecp_g_discode_sensor_test();

	bool first_step();
	bool next_step();
private:
	mrrocpp::ecp_mp::sensor::discode::discode_sensor &ds;
	int jjj;
};

} // namespace generator

} // namespace common

} // namespace ecp

} // namespace mrrocpp

#endif /* ECP_G_DISCODE_SENSOR_TEST_H_ */
