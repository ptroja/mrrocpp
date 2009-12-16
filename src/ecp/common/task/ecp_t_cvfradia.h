/*!
 * \file task/ecp_t_cvfradia.h
 * \brief Class responsible for communication with cvFraDIA (testing purposes).
 * - class declaration
 * \author tkornuta
 * \date 17.03.2008
 */

#if !defined(_ECP_T_CVFRADIA_H)
#define _ECP_T_CVFRADIA_H


#include "ecp/common/task/ecp_task.h"
#include "ecp/common/generator/ecp_g_cvfradia.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace task {


/*!
 * \class ecp_task_cvfradia
 * \brief Class responsible for communication with cvFraDIA (testing purposes).
 * \author tkornuta
 */
class cvfradia: public common::task::task  {
protected:
	/*!
      * Generator used for communication with cvFraDIA.
      */
	generator::cvfradia* cvg;

public:
	/*!
      * Constructor.
      */
	cvfradia(lib::configurator &_config);

	/*!
      * Main algorithm loop. Retrieves information from cvFraDIA.
      */
	void main_task_algorithm(void);
};

} // namespace task
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif
