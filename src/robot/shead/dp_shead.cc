/*!
 * @file
 * @brief File contains dp_shead class definition for SwarmItFix head
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup shead
 */

#include <cmath>
#include <cstring>
#include <ostream>

#include "dp_shead.h"

namespace mrrocpp {
namespace lib {
namespace shead {

reply::reply() :
		solidification_state(SOLIDIFICATION_STATE_INTERMEDIATE), vacuum_state(VACUUM_STATE_INTERMEDIATE)
{
}

reply & reply::operator=(const reply & wzor)
{
	// operator przypisania
	// parametry macierzy przyjmuja wartosc jak parametry macierzy podanej jako argumet

	if (this == &wzor)
		return *this;

	solidification_state = wzor.solidification_state;

	vacuum_state = wzor.vacuum_state;

	return *this;
}

} // namespace robot
} // namespace mp
} // namespace mrrocpp

