#include "SingleManipulation.h"

namespace mrrocpp {
namespace mp {
namespace common {

SingleManipulation::SingleManipulation(CUBE_COLOR face_to_turn_l, CUBE_TURN_ANGLE turn_angle_l)
	: face_to_turn(face_to_turn_l), turn_angle(turn_angle_l)
{
}

CUBE_TURN_ANGLE read_cube_turn_angle(char input_char)
{
	switch (input_char)
	{
		case '0':
			return CL_0;
			break;
		case '1':
			return CL_90;
			break;
		case '2':
			return CL_180;
			break;
		case '3':
			return CCL_90;
			break;
		default:
			return UKNOWN_TURN_ANGLE;
			break;
	}
}

} // namespace common
} // namespace mp
} // namespace mrrocpp

