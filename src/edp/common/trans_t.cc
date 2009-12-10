// ------------------------------------------------------------------------
// transformation thread by Y
// ------------------------------------------------------------------------

/********************************* INCLUDES *********************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "lib/mis_fun.h"
#include "edp/common/trans_t.h"

/********************************* GLOBALS **********************************/

namespace mrrocpp {
namespace edp {
namespace common {


trans_t::trans_t(effector& _master) :
	master (_master)
{
	// semafory do komunikacji miedzy EDP_MASTER a EDP_TRANS
	sem_init(&master_to_trans_t_sem, 0, 0);
	sem_init(&trans_t_to_master_sem, 0, 0);
}

trans_t::~trans_t()
{
	// semafory do komunikacji miedzy EDP_MASTER a EDP_TRANS
	sem_destroy(&master_to_trans_t_sem);
	sem_destroy(&trans_t_to_master_sem);
}

int trans_t::master_to_trans_t_order(MT_ORDER nm_task, int nm_tryb)
{ // zlecenie z watku master dla trans_t
	trans_t_task = nm_task; // force, arm etc.
	trans_t_tryb = nm_tryb; // tryb dla zadania

	// odwieszenie watku transformation
	sem_trywait(&master_to_trans_t_sem);
	sem_post(&master_to_trans_t_sem);

	// oczekiwanie na zwolniene samafora przez watek trans_t
	sem_wait(&trans_t_to_master_sem); // oczekiwanie na zezwolenie ruchu od edp_master

	// sekcja sprawdzajaca czy byl blad w watku transforamation i ew. rzucajaca go w watku master

	switch (error) {
		case NonFatal_erroR_1:
			throw *(kinematic::common::transformer_error::NonFatal_error_1*)(error_pointer);
			break;
		case NonFatal_erroR_2:
			throw *(kinematic::common::transformer_error::NonFatal_error_2*)(error_pointer);
			break;
		case NonFatal_erroR_3:
			throw *(kinematic::common::transformer_error::NonFatal_error_3*)(error_pointer);
			break;
		case NonFatal_erroR_4:
			throw *(kinematic::common::transformer_error::NonFatal_error_4*)(error_pointer);
			break;
		case Fatal_erroR:
			throw *(kinematic::common::transformer_error::Fatal_error*)(error_pointer);
			break;
		case System_erroR:
			throw *(System_error*)(error_pointer);
			break;
		default:
			break;
	}

	return 1;
}

// oczekiwanie na semafor statusu polecenia z trans_t
int trans_t::master_wait_for_trans_t_order_status()
{
	// oczekiwanie na odpowiedz z watku transformation
	return sem_wait(&trans_t_to_master_sem);
}

// oczekiwanie na semafor statusu polecenia z trans_t
int trans_t::trans_t_to_master_order_status_ready()
{
	// odwieszenie watku new master
	sem_trywait(&trans_t_to_master_sem);
	return sem_post(&trans_t_to_master_sem);// odwieszenie watku edp_master
}

// oczekiwanie na semafor statusu polecenia z trans_t
int trans_t::trans_t_wait_for_master_order()
{
	// oczekiwanie na rozkaz z watku master
	return sem_wait(&master_to_trans_t_sem);
}




} // namespace common
} // namespace edp
} // namespace mrrocpp

