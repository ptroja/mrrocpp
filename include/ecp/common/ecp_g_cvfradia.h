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
namespace generator {

/*!
 * \class ecp_cvfradia_generator
 * \brief Generator responsible for communication with cvFraDIA (testing purposes).
 * \author tkornuta
 */
class cvfradia : public common::generator::base
{
public:
	/*!
      * Constructor.
      */
	cvfradia(common::task::base& _ecp_task)
	  : base(_ecp_task)
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

} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp


#endif
