/*!
 * \file task/ecp_t_pcbird.h
 * \brief Class responsible for communication with pcbird (testing purposes).
 * - class declaration
 * \author tkornuta
 * \date 17.03.2008
 */

#if !defined(_ECP_T_PCBIRD_H)
#define _ECP_T_PCBIRD_H


#include "ecp/common/task/ecp_task.h"
#include "ecp/common/generator/ecp_g_cvfradia.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace task {


/*!
 * \class ecp_task_pcbird
 * \brief Class responsible for communication with pcbird (testing purposes).
 * \author tkornuta
 */
class pcbird: public common::task::task  {
protected:
	/*!
      * Generator used for communication with pcbird.
      */
	generator::cvfradia* cvg;

public:
	/*!
      * Constructor.
      */
	pcbird(lib::configurator &_config);

	/*!
      * Main algorithm loop. Retrieves information from pcbird.
      */
	void main_task_algorithm(void);

};

} // namespace task
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif
