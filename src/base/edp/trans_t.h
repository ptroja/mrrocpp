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

class trans_t : public boost::noncopyable
{
protected:
	boost::thread thread_id;

	MT_ORDER trans_t_task;
	int trans_t_tryb;
	lib::c_buffer instruction;

	virtual void operator()() = 0;

public:
	lib::condition_synchroniser master_to_trans_synchroniser;
	lib::condition_synchroniser trans_t_to_master_synchroniser;

	ERROR_TYPE error;

	// wskaznik na bledy (rzutowany na odpowiedni blad)
	void* error_pointer;

	virtual ~trans_t();

	void master_to_trans_t_order(MT_ORDER nm_task, int nm_tryb, const lib::c_buffer& _instruction);
};
/**************************** trans_t *****************************/

} // namespace common
} // namespace edp
} // namespace mrrocpp

#endif
