#include "ecp/common/ecp_generator_t.h"

ecp_generator_t::ecp_generator_t (ecp_task& _ecp_task, bool _is_robot_active): 
	ecp_generator (_ecp_task, _is_robot_active){};


// ----------------------------------------------------------------------------------------------
// ---------------------------------    metoda	first_step -------------------------------------
// ----------------------------------------------------------------------------------------------

bool ecp_generator_t::first_step (  ) {
	return next_step ();

}; // end: bool conveyor_generator_t::first_step ( )

// ----------------------------------------------------------------------------------------------
// -----------------------------------  metoda	next_step --------------------------------------
// ----------------------------------------------------------------------------------------------

bool ecp_generator_t::next_step (  ) {

  // Kopiowanie danych z bufora przyslanego z EDP do
  // obrazu danych wykorzystywanych przez generator
// the_robot->get_reply();
  
  // by Y - Przepisanie przyslanej z EDP pozycji do MP
  the_robot->copy_edp_to_mp_buffer (ecp_t.ecp_reply.ecp_reply.reply_package);
  the_robot->copy_mp_to_edp_buffer (ecp_t.mp_command.mp_package.instruction);
   return true;
  
}; // end: bool conveyor_generator_t::next_step ( )
