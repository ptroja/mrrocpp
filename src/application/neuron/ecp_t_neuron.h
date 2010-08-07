/*
 * ecp_t_neuron.h
 *
 *  Created on: May 13, 2010
 *      Author: tbem
 */

#ifndef ECP_T_NEURON_H_
#define ECP_T_NEURON_H_

#include "base/ecp/ecp_task.h"
#include "generator/ecp/ecp_g_newsmooth.h"
#include "generator/ecp/ecp_g_constant_velocity.h"
#include "ecp_g_neuron_generator.h"
#include "generator/ecp/ecp_g_sleep.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot {
namespace task {

class Neuron: public common::task::task{

	private:
		common::generator::newsmooth* smoothGenerator;
		common::generator::neuron_generator* neuronGenerator;
		ecp_mp::sensor::neuron_sensor* neuronSensor;

	public:
		Neuron(lib::configurator &_config);
		~Neuron();
		void mp_2_ecp_next_state_string_handler(void);
		void ecp_stop_accepted_handler();
};

} // namespace task
} // namespace irp6ot
} // namespace ecp
} // namespace mrrocpp

#endif /* ECP_T_NEURON_H_ */
