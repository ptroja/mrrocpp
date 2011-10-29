/*!
 * @file
 * @brief File contains sub_task class definition
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup spring_contact
 */

#include "ecp_g_spring_contact.h"
#include "ecp_st_spring_contact.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace sub_task {

spring_contact::spring_contact(task::task &_ecp_t) :
		sub_task(_ecp_t)
{
	scg = new generator::spring_contact(_ecp_t, 5);
}

void spring_contact::conditional_execution()
{

	scg->Move();
}

} // namespace task

} // namespace common
} // namespace ecp
} // namespace mrrocpp
