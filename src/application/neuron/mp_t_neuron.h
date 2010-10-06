/**
 * @file mp_t_neuron.h
 * @brief Header file for neuron class.
 * @author Tomasz Bem (mebmot@wp.pl)
 * @ingroup neuron
 * @date 25.06.2010
 */

#ifndef MP_T_NEURON_H_
#define MP_T_NEURON_H_

namespace mrrocpp {
namespace mp {
namespace task {

/**
 * @brief neuron task class declaration.
 * @detials Taks for optimization trajectory with usage of neural networks.
 */
class neuron : public task
{
public:
	neuron(lib::configurator &_config);
	void main_task_algorithm(void);
	/// utworzenie robotow
	void create_robots(void);
	virtual ~neuron();
};

} //task
} //mp
} //mrrocpp


#endif /* MP_T_NEURON_H_ */
