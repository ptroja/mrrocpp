/*!
 * @file
 * @brief File contains dp_sbenchclass definition for SwarmItFix Head
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup sbench
 */

#include <cmath>
#include <cstring>
#include <ostream>

#include "dp_sbench.h"

namespace mrrocpp {
namespace lib {
namespace sbench {

pins_buffer::pins_buffer()
{
}

void pins_buffer::set_zeros()
{
	for (int i = 0; i < lib::sbench::NUM_OF_PINS; i++) {

		pins_state[i] = 0;
	}
}

pins_buffer & pins_buffer::operator=(const pins_buffer & wzor)
{
	// operator przypisania
	// parametry macierzy przyjmuja wartosc jak parametry macierzy podanej jako argumet

	if (this == &wzor)
		return *this;

	memcpy(pins_state, wzor.pins_state, sizeof(pins_state));

	return *this;
}

} // namespace robot
} // namespace mp
} // namespace mrrocpp

