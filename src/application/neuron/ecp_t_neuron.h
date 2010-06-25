/*
 * ecp_t_neuron.h
 *
 *  Created on: May 13, 2010
 *      Author: tbem
 */

#ifndef ECP_T_NEURON_H_
#define ECP_T_NEURON_H_

#include "base/ecp/ecp_task.h"
#include "generator/ecp/ecp_g_smooth.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot {
namespace task {

class Neuron: public common::task::task{

	private:
		common::generator::smooth* sgen2;				//smooth movement generator

	public:
		Neuron(lib::configurator &_config);
		~Neuron();
		//void main_task_algorithm(void);
		void mp_2_ecp_next_state_string_handler(void);
};

}
} // namespace irp6ot
} // namespace ecp
} // namespace mrrocpp

#endif /* ECP_T_NEURON_H_ */
