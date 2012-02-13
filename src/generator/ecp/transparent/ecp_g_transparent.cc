/*!
 * @file
 * @brief File contains ecp transparent generator definition
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup generators
 */

#include "base/ecp/ecp_task.h"
#include "base/ecp/ecp_robot.h"
#include "ecp_g_transparent.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

transparent::transparent(common::task::task& _ecp_task) :
		common::generator::generator(_ecp_task)
{
	throw_kinematics_exceptions = true;
	generator_name = ecp_mp::generator::ECP_GEN_TRANSPARENT;
}

// ----------------------------------------------------------------------------------------------
// ---------------------------------    metoda	first_step -------------------------------------
// ----------------------------------------------------------------------------------------------

bool transparent::first_step()
{
	//sr_ecp_msg.message(lib::NON_FATAL_ERROR, "transparent::first_step()");

	if (the_robot)
		the_robot->communicate_with_edp = false;
	ecp_t.continuous_coordination = true;
	return true;
}

// ----------------------------------------------------------------------------------------------
// -----------------------------------  metoda	next_step --------------------------------------
// ----------------------------------------------------------------------------------------------

bool transparent::next_step()
{
	//sr_ecp_msg.message(lib::NON_FATAL_ERROR, "transparent::next_step()");
	// Kopiowanie danych z bufora przyslanego z EDP do
	// obrazu danych wykorzystywanych przez generator
	// the_robot->get_reply();
	if (the_robot)
		the_robot->communicate_with_edp = true;

	// by Y - Przepisanie przyslanej z EDP pozycji do MP
	the_robot->copy_edp_to_mp_buffer(ecp_t.ecp_reply.reply_package);
	the_robot->copy_mp_to_edp_buffer(ecp_t.mp_command.instruction);

	return true;
}

void transparent::execute_motion(void)
{
	// Zlecenie wykonania ruchu przez robota jest to polecenie dla EDP

	// komunikacja wlasciwa
	the_robot->send();
	if (the_robot->reply_package.reply_type == lib::ERROR) {

		the_robot->query();
		BOOST_THROW_EXCEPTION(exception::nfe_r() << lib::exception::mrrocpp_error0(EDP_ERROR));

	}
	the_robot->query();

	if (the_robot->reply_package.reply_type == lib::ERROR) {

		switch (the_robot->reply_package.error_no.error0)
		{
			case BEYOND_UPPER_D0_LIMIT:
			case BEYOND_UPPER_THETA1_LIMIT:
			case BEYOND_UPPER_THETA2_LIMIT:
			case BEYOND_UPPER_THETA3_LIMIT:
			case BEYOND_UPPER_THETA4_LIMIT:
			case BEYOND_UPPER_THETA5_LIMIT:
			case BEYOND_UPPER_THETA6_LIMIT:
			case BEYOND_UPPER_THETA7_LIMIT:
			case BEYOND_LOWER_D0_LIMIT:
			case BEYOND_LOWER_THETA1_LIMIT:
			case BEYOND_LOWER_THETA2_LIMIT:
			case BEYOND_LOWER_THETA3_LIMIT:
			case BEYOND_LOWER_THETA4_LIMIT:
			case BEYOND_LOWER_THETA5_LIMIT:
			case BEYOND_LOWER_THETA6_LIMIT:
			case BEYOND_LOWER_THETA7_LIMIT:
				if (throw_kinematics_exceptions) {
					BOOST_THROW_EXCEPTION(exception::nfe_r() << lib::exception::mrrocpp_error0(EDP_ERROR));
				}
				break;
			default:
				BOOST_THROW_EXCEPTION(exception::nfe_r() << lib::exception::mrrocpp_error0(EDP_ERROR));
				break;

		} /* end: switch */
	}
}

void transparent::conditional_execution()
{
	throw_kinematics_exceptions = (bool) ecp_t.mp_command.ecp_next_state.variant;

	Move();
}

} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp

