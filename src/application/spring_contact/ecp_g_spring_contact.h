#if !defined(_ECP_GEN_SPRING_CONTACT_H)
#define _ECP_GEN_SPRING_CONTACT_H

/*!
 * @file
 * @brief File contains ecp_generator class declaration
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup spring_contact
 */

#include "generator/ecp/ecp_g_teach_in.h"
#include "base/lib/mrmath/mrmath.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

/**
 * @brief degrees to radians factor constant
 */
const double DEGREES_TO_RADIANS = 57.295780;

/*!
 * @brief Generator that getting into contact with spring
 *
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 * @ingroup spring_contact
 */
class spring_contact : public common::generator::generator
{
protected:

	/**
	 * @brief number of steps in each macrostep
	 */
	const int step_no;

	/**
	 * @brief manipualtor tool frame
	 */
	lib::Homog_matrix tool_frame;

public:

	/**
	 * @brief Constructor
	 * @param _ecp_task ecp task object reference.
	 */
	spring_contact(common::task::task& _ecp_task, int step);

	bool first_step();
	bool next_step();

};

} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif
