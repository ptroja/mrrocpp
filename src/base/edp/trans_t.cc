// ------------------------------------------------------------------------
// transformation thread by Y
// ------------------------------------------------------------------------

/********************************* INCLUDES *********************************/
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "base/edp/trans_t.h"

/********************************* GLOBALS **********************************/

namespace mrrocpp {
namespace edp {
namespace common {

trans_t::trans_t(effector& _master) :
	master(_master), master_to_trans_synchroniser(), trans_t_to_master_synchroniser()
{
}

trans_t::~trans_t()
{
}

void trans_t::master_to_trans_t_order(MT_ORDER nm_task, int nm_tryb, const lib::c_buffer& _instruction)
{ // zlecenie z watku master dla trans_t
	trans_t_task = nm_task; // force, arm etc.
	trans_t_tryb = nm_tryb; // tryb dla zadania
	instruction = _instruction;

	// odwieszenie watku transformation
	master_to_trans_synchroniser.command();

	// oczekiwanie na zwolniene samafora przez watek trans_t
	trans_t_to_master_synchroniser.wait();

	// sekcja sprawdzajaca czy byl blad w watku transforamation i ew. rzucajaca go w watku master

	switch (error)
	{
		case NonFatal_erroR_1:
			throw *(NonFatal_error_1*) (error_pointer);
			break;
		case NonFatal_erroR_2:
			throw *(NonFatal_error_2*) (error_pointer);
			break;
		case NonFatal_erroR_3:
			throw *(NonFatal_error_3*) (error_pointer);
			break;
		case NonFatal_erroR_4:
			throw *(NonFatal_error_4*) (error_pointer);
			break;
		case Fatal_erroR:
			throw *(Fatal_error*) (error_pointer);
			break;
		case System_erroR:
			throw *(System_error*) (error_pointer);
			break;
		default:
			break;
	}
}

} // namespace common
} // namespace edp
} // namespace mrrocpp

