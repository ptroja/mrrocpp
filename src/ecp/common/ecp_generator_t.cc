#include "ecp/common/ecp_generator_t.h"

ecp_generator_t::ecp_generator_t(ecp_task& _ecp_task) :
	ecp_generator(_ecp_task)
{
	copy_edp_buffers_in_move=false;
}


// ----------------------------------------------------------------------------------------------
// ---------------------------------    metoda	first_step -------------------------------------
// ----------------------------------------------------------------------------------------------

bool ecp_generator_t::first_step()
{
	communicate_with_edp=false;
	return true;
}

// ----------------------------------------------------------------------------------------------
// -----------------------------------  metoda	next_step --------------------------------------
// ----------------------------------------------------------------------------------------------

bool ecp_generator_t::next_step()
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
