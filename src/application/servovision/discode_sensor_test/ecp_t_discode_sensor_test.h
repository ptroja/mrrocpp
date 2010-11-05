/*
 * ecp_t_discode_sensor_test.h
 *
 *  Created on: Nov 4, 2010
 *      Author: mboryn
 */

#ifndef ECP_T_DISCODE_SENSOR_TEST_H_
#define ECP_T_DISCODE_SENSOR_TEST_H_

#include "base/ecp/ecp_task.h"
#include "ecp_g_discode_sensor_test.h"

namespace mrrocpp {

namespace ecp {

namespace discode_sensor_test {

namespace task {

class ecp_t_discode_sensor_test : public mrrocpp::ecp::common::task::task
{
public:
	ecp_t_discode_sensor_test(mrrocpp::lib::configurator& config);
	virtual ~ecp_t_discode_sensor_test();
	void main_task_algorithm();
};

} // namespace task {

} // namespace discode_sensor_test

} // namespace ecp

}  // namespace mrrocpp

#endif /* ECP_T_DISCODE_SENSOR_TEST_H_ */
