#ifndef MP_GEN_SET_NEXT_ECPS_STATE_H_
#define MP_GEN_SET_NEXT_ECPS_STATE_H_

/*!
 * @file
 * @brief File contains mp set_next_ecps_state generator declaration
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup mp
 */

#include "base/mp/generator/mp_generator.h"

// generator for setting the next ecps state

namespace mrrocpp {
namespace mp {
namespace generator {

class set_next_ecps_state : public generator
{
protected:
	lib::ecp_next_state_t ecp_next_state;

public:
	set_next_ecps_state(task::task& _mp_task);

	void
			configure(std::string l_mp_2_ecp_next_state, int l_mp_2_ecp_next_state_variant, const char* l_mp_2_ecp_next_state_string, int str_len);
	void configure(const lib::playerpos_goal_t &_goal);

	bool first_step();
	bool next_step();
};

} // namespace generator
} // namespace mp
} // namespace mrrocpp

#endif /*MP_GENERATORS_H_*/
