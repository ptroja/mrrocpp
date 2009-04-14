#include "ecp/common/ecp_generator_t.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

transparent::transparent(common::task::base& _ecp_task) :
	base(_ecp_task)
{
	copy_edp_buffers_in_move=false;
	throw_kinematics_exceptions = true;
}


// ----------------------------------------------------------------------------------------------
// ---------------------------------    metoda	first_step -------------------------------------
// ----------------------------------------------------------------------------------------------

bool transparent::first_step()
{
	communicate_with_edp=false;
	return true;
}

// ----------------------------------------------------------------------------------------------
// -----------------------------------  metoda	next_step --------------------------------------
// ----------------------------------------------------------------------------------------------

bool transparent::next_step()
{
	// Kopiowanie danych z bufora przyslanego z EDP do
	// obrazu danych wykorzystywanych przez generator
	// the_robot->get_reply();	
		communicate_with_edp=true;

	// by Y - Przepisanie przyslanej z EDP pozycji do MP
	the_robot->copy_edp_to_mp_buffer(ecp_t.ecp_reply.reply_package);
	the_robot->copy_mp_to_edp_buffer(ecp_t.mp_command.instruction);

	return true;
}


void transparent::execute_motion(void)
{
	// Zlecenie wykonania ruchu przez robota jest to polecenie dla EDP
	/*
	 // maskowanie sygnalu SIGTERM
	 // w celu zapobierzenia przerwania komunikacji ECP z EDP pomiedzy SET a QUERY - usuniete

	 sigset_t set;

	 sigemptyset( &set );
	 sigaddset( &set, SIGTERM );

	 if  (sigprocmask( SIG_SETMASK, &set, NULL)==-1)
	 {
	 printf ("blad w ECP procmask signal\n");
	 }
	 */
	// komunikacja wlasciwa
	the_robot->send();
	if (the_robot->reply_package.reply_type == lib::ERROR) {

		the_robot->query();
		throw ecp_robot::ECP_error (lib::NON_FATAL_ERROR, EDP_ERROR);

	}
	the_robot->query();

	/*
	 // odmaskowanie sygnalu SIGTERM

	 sigemptyset( &set );

	 if  (sigprocmask( SIG_SETMASK, &set, NULL)==-1)
	 {
	 printf ("blad w ECP procmask signal\n");
	 }
	 */
	if (the_robot->reply_package.reply_type == lib::ERROR) {
		
		
		
		switch ( the_robot->reply_package.error_no.error0 ) {
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
				if (throw_kinematics_exceptions) 
				{
					throw ecp_robot::ECP_error (lib::NON_FATAL_ERROR, EDP_ERROR);
				}
				
			break;
			default:
				throw ecp_robot::ECP_error (lib::NON_FATAL_ERROR, EDP_ERROR);
			break;

		} /* end: switch */
		
	
	}
}

} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp

