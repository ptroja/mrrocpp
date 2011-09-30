#ifndef SINGLE_MANIPULATION_H_
#define SINGLE_MANIPULATION_H_

#include "CubeState.h"

namespace mrrocpp {
namespace mp {
namespace common {

// zbior obejmujacy mozliwe stany kostki
enum CUBE_TURN_ANGLE {UKNOWN_TURN_ANGLE, CL_180, CL_90, CL_0, CCL_90};

CUBE_TURN_ANGLE read_cube_turn_angle (char input_char);

class SingleManipulation
{
public:
	const CUBE_COLOR face_to_turn;
	const CUBE_TURN_ANGLE turn_angle;

	SingleManipulation(CUBE_COLOR face_to_turn_l, CUBE_TURN_ANGLE turn_angle_l);
};

} // namespace common
} // namespace mp
} // namespace mrrocpp


#endif /*SINGLE_MANIPULATION_H_*/

