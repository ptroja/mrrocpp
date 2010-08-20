/*
 * mp_t_neuron.h
 *
 *  Created on: Jun 25, 2010
 *      Author: tbem
 */

#ifndef MP_T_NEURON_H_
#define MP_T_NEURON_H_

#include "base/mp/mp.h"
#include "base/mp/MP_main_error.h"

namespace mrrocpp {
namespace mp {
namespace task {

class neuron: public task {
public:
	neuron(lib::configurator &_config);
	void main_task_algorithm(void);
	virtual ~neuron();
};

} //task
} //mp
} //mrrocpp


#endif /* MP_T_NEURON_H_ */
