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

#include "base/lib/exception.h"

#include "base/lib/condition_synchroniser.h"

using namespace mrrocpp::lib::exception;

namespace mrrocpp {
namespace edp {
namespace common {

/**************************** trans_t *****************************/
template <typename COMMAND_T = lib::c_buffer>
class trans_t : public boost::noncopyable
{
protected:
	boost::thread thread_id;

	typedef struct _master_to_transformer_cmd
	{
		MT_ORDER trans_t_task;
		int trans_t_tryb;
		COMMAND_T instruction;
	} master_to_transformer_cmd_t;

	master_to_transformer_cmd_t tmp_cmd, current_cmd;

	virtual void operator()() = 0;

public:
	lib::condition_synchroniser master_to_trans_synchroniser;
	lib::condition_synchroniser trans_t_to_master_synchroniser;

	ERROR_TYPE error;

	// wskaznik na bledy (rzutowany na odpowiedni blad)
	void* error_pointer;

	virtual ~trans_t()
	{
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
};
/**************************** trans_t *****************************/

} // namespace common
} // namespace edp
} // namespace mrrocpp

#endif
