/*!
 * \file ecp_g_cvfradia.h
 * \brief Generator responsible for communication with cvFraDIA (testing purposes).
 * - class declaration
 * \author tkornuta
 * \date 17.03.2008
 */

#if !defined(_ECP_G_CVFRADIA_H)
#define _ECP_G_CVFRADIA_H


#include "ecp/common/ecp_task.h"
#include "ecp/common/ecp_generator.h"


namespace mrrocpp {
namespace ecp {
namespace common {

/*!
 * \class ecp_cvfradia_generator
 * \brief Generator responsible for communication with cvFraDIA (testing purposes).
 * \author tkornuta
 */
class ecp_cvfradia_generator : public common::ecp_generator
{
public:
	/*!
      * Constructor.
      */
	ecp_cvfradia_generator(common::task::ecp_task& _ecp_task)
	  : ecp_generator(_ecp_task)
	{  }

	/*!
      * First step method..
      */
    bool first_step ();

	/*!
      * Next step method.
      */
    bool next_step ();
};

} // namespace common
} // namespace ecp
} // namespace mrrocpp


#endif
