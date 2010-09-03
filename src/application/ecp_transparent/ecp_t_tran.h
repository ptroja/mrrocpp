#if !defined(_ECP_T_TRAN_H)
#define _ECP_T_TRAN_H

/*!
 * @file
 * @brief File contains ecp transparent task declaration
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup
 */

#include "base/ecp/ecp_task.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace task {

class tran : public common::task::task
{

public:
	// KONSTRUKTORY
	tran(lib::configurator &_config);

	// methods for ECP template to redefine in concrete classes
	void main_task_algorithm(void);
};

} // namespace task
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif
