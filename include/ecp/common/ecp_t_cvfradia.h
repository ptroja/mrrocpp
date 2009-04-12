/*!
 * \file ecp_t_cvfradia.h
 * \brief Class responsible for communication with cvFraDIA (testing purposes).
 * - class declaration
 * \author tkornuta
 * \date 17.03.2008
 */

#if !defined(_ECP_T_CVFRADIA_H)
#define _ECP_T_CVFRADIA_H


#include "ecp/common/ecp_task.h"
#include "ecp/common/ecp_g_cvfradia.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace task {


/*!
 * \class ecp_task_cvfradia
 * \brief Class responsible for communication with cvFraDIA (testing purposes).
 * \author tkornuta
 */
class ecp_task_cvfradia: public common::task::ecp_task  {
protected:
	/*!
      * Generator used for communication with cvFraDIA.
      */
	ecp_cvfradia_generator* cvg;
	
public:
	/*!
      * Constructor.
      */
	ecp_task_cvfradia(configurator &_config)
	  : ecp_task(_config)
	{ }
	
	/*!
      * Initialize task - robot, sensors and generators.
      */
	void task_initialization(void);

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
