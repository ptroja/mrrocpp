// -------------------------------------------------------------------------
//                                   edp.h
// Definicje struktur danych i metod dla procesu EDP
//
// Ostatnia modyfikacja: 16.04.98
// -------------------------------------------------------------------------

#ifndef __TRANS_T_H
#define __TRANS_T_H

#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"

#include "base/edp/edp_typedefs.h"

#include "base/lib/exception.h"

#include "base/lib/condition_synchroniser.h"

#include <boost/utility.hpp>
#include <boost/thread/thread.hpp>

using namespace mrrocpp::lib::exception;

namespace mrrocpp {
namespace edp {
namespace common {

class effector;

/**************************** trans_t *****************************/

class trans_t : public boost::noncopyable
{
private:

	effector &master;

protected:
	boost::thread *thread_id;
	lib::c_buffer instruction;

public:
	lib::condition_synchroniser master_to_trans_synchroniser;
	lib::condition_synchroniser trans_t_to_master_synchroniser;

	MT_ORDER trans_t_task;
	int trans_t_tryb;
	ERROR_TYPE error;

	virtual void operator()() = 0;

	// wskaznik na bledy (rzutowany na odpowiedni blad)
	void* error_pointer;

	trans_t(effector& _master);
	virtual ~trans_t();

	void master_to_trans_t_order(MT_ORDER nm_task, int nm_tryb, const lib::c_buffer& _instruction);

};
/**************************** trans_t *****************************/

} // namespace common
} // namespace edp
} // namespace mrrocpp

#endif
