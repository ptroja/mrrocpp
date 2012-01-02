/*!
 * @file
 * @brief File contains dp_smb class definition for SwarmItFix smb
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup smb
 */

#include <cmath>
#include <cstring>
#include <ostream>

#include "dp_smb.h"

namespace mrrocpp {
namespace lib {
namespace smb {

leg_reply & leg_reply::operator=(const leg_reply & wzor)
{
	// operator przypisania
	// parametry macierzy przyjmuja wartosc jak parametry macierzy podanej jako argumet

	if (this == &wzor)
		return *this;

	is_in = wzor.is_in;
	is_out = wzor.is_out;
	is_attached = wzor.is_attached;

	return *this;
}

multi_leg_reply_td & multi_leg_reply_td::operator=(const multi_leg_reply_td & wzor)
{
	// operator przypisania
	// parametry macierzy przyjmuja wartosc jak parametry macierzy podanej jako argumet

	if (this == &wzor)
		return *this;

	for (int i = 0; i < LEG_CLAMP_NUMBER; ++i) {
		leg[i] = wzor.leg[i];
	}

	return *this;
}

} // namespace robot
} // namespace mp
} // namespace mrrocpp

