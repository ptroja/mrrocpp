// -------------------------------------------------------------------------
//                                   edp.h
// Definicje struktur danych i metod dla procesu EDP
//
// Ostatnia modyfikacja: 16.04.98
// -------------------------------------------------------------------------

#ifndef __TRANS_T_H
#define __TRANS_T_H

#include <boost/utility.hpp>
#include <boost/thread/thread.hpp>

#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"

#include "base/edp/edp_typedefs.h"

#include "edp_exceptions.h"

#include "base/lib/condition_synchroniser.h"

using namespace mrrocpp::edp::exception;

namespace mrrocpp {
namespace edp {
namespace common {

/**************************** trans_t *****************************/
template <typename COMMAND_T = lib::c_buffer>
class trans_t : public boost::noncopyable
{
protected:
	typedef struct _master_to_transformer_cmd
	{
		MT_ORDER trans_t_task;
		int trans_t_tryb;
		COMMAND_T instruction;
	} master_to_transformer_cmd_t;

	master_to_transformer_cmd_t tmp_cmd, current_cmd;

	virtual void operator()() = 0;

public:
	boost::thread thread_id;

	lib::condition_synchroniser master_to_trans_synchroniser;
	lib::condition_synchroniser trans_t_to_master_synchroniser;

	//wskaznik na nowe bledy boost
	boost::exception_ptr error;

	virtual ~trans_t()
	{
		thread_id.interrupt();
		thread_id.join();
	}

	void master_to_trans_t_order(MT_ORDER nm_task, int nm_tryb, const COMMAND_T& _instruction)
	{
		tmp_cmd.trans_t_task = nm_task; // force, arm etc.
		tmp_cmd.trans_t_tryb = nm_tryb; // tryb dla zadania
		tmp_cmd.instruction = _instruction;

		// odwieszenie watku transformation
		master_to_trans_synchroniser.command();

		// oczekiwanie na zwolniene samafora przez watek trans_t
		trans_t_to_master_synchroniser.wait();

		// sekcja sprawdzajaca czy byl blad w watku transforamation i ew. rzucajaca go w watku master

		if (error) {
			boost::rethrow_exception(error);
		}
	}
};
/**************************** trans_t *****************************/

} // namespace common
} // namespace edp
} // namespace mrrocpp

#endif
