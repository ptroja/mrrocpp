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

/*!
 * @brief Generator that sends NEXT_STATE command with parameters to coordinated robots.
 *
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 * @ingroup mp
 */
class set_next_ecps_state : public generator
{
protected:
	/**
	 * @brief next state structure send to ECPs
	 */
	lib::ecp_next_state_t ecp_next_state;

	/*!
	 * @brief stl map of all mp robots that have not reply
	 */
	common::robots_t robots_to_reply;

public:

	/**
	 * @brief Constructor
	 * @param _mp_task mp task object reference.
	 */
	set_next_ecps_state(task::task& _mp_task);

	/**
	 * @brief sets ecp_next_state structure (common variant)
	 * @param l_mp_2_ecp_next_state next state label
	 * @param l_mp_2_ecp_next_state_variant next state variant (int)
	 * @param l_mp_2_ecp_next_state_string next state string to store extra parameters
	 * @param str_len above string length
	 */

	template <typename BUFFER_TYPE>
	void configure(const std::string & l_mp_2_ecp_next_state, int l_mp_2_ecp_next_state_variant, const BUFFER_TYPE & l_mp_2_ecp_next_state_data)
	{
		ecp_next_state.next_state = l_mp_2_ecp_next_state;
		ecp_next_state.variant = l_mp_2_ecp_next_state_variant;
		ecp_next_state.sg_buf.set(l_mp_2_ecp_next_state_data);
	}

	bool first_step();
	bool next_step();
};

} // namespace generator
} // namespace mp
} // namespace mrrocpp

#endif /*MP_GENERATORS_H_*/
