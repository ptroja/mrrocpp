#include "ecp/common/generator/ecp_g_transparent.h"

#include "lib/exception.h"
#include <boost/throw_exception.hpp>

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

transparent::transparent(common::task::task& _ecp_task) :
	generator(_ecp_task)
{
	throw_kinematics_exceptions = true;
}

// ----------------------------------------------------------------------------------------------
// ---------------------------------    metoda	first_step -------------------------------------
// ----------------------------------------------------------------------------------------------

bool transparent::first_step()
{
	if (the_robot) the_robot->communicate_with_edp = false;
	ecp_t.continuous_coordination = true;
	return true;
}

// ----------------------------------------------------------------------------------------------
// -----------------------------------  metoda	next_step --------------------------------------
// ----------------------------------------------------------------------------------------------

bool transparent::next_step()
{
	// Przepisanie przyslanej z EDP pozycji do MP
	the_robot->copy_edp_to_mp_buffer(ecp_t.ecp_reply.reply_package);
	ecp_t.ecp_reply_buffer.Set(ecp_t.ecp_reply);

	if(ecp_t.mp_command_buffer.isFresh()) {
		ecp_t.mp_command_buffer.Get(ecp_t.mp_command);
		the_robot->copy_mp_to_edp_buffer(ecp_t.mp_command.instruction);

		if (the_robot) the_robot->communicate_with_edp = true;

		if (ecp_t.mp_command.command == lib::END_MOTION ||
			ecp_t.mp_command.command == lib::NEXT_STATE	||
			ecp_t.mp_command.command == lib::STOP)

			return false;
	} else {
		if (the_robot) the_robot->communicate_with_edp = false;
	}

	return true;
}

void transparent::execute_motion(void)
{
	// Zlecenie wykonania ruchu przez robota jest to polecenie dla EDP

	// komunikacja wlasciwa
	assert(	ecp_t.mp_command.instruction.instruction_type == lib::SET_GET ||
			ecp_t.mp_command.instruction.instruction_type == lib::GET);
	the_robot->send();
	if (the_robot->reply_package.reply_type == lib::ERROR) {

		the_robot->query();

		BOOST_THROW_EXCEPTION(
				lib::exception::NonFatal_error() <<
				lib::exception::error_code(EDP_ERROR) <<
				lib::exception::err0(the_robot->reply_package.error_no.error0) <<
				lib::exception::err1(the_robot->reply_package.error_no.error1)
		);
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
					BOOST_THROW_EXCEPTION(
							lib::exception::NonFatal_error() <<
							lib::exception::error_code(EDP_ERROR)
					);
				}
				break;
			default:
				BOOST_THROW_EXCEPTION(
						lib::exception::NonFatal_error() <<
						lib::exception::error_code(EDP_ERROR)
				);
				break;

		} /* end: switch */
	}
}

} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp

