#ifndef MP_R_motor_driven_H_
#define MP_R_motor_driven_H_

#include "lib/impconst.h"
#include "base/mp/mp.h"
#include "base/mp/mp_robot.h"

#include "robot/irp6_mechatronika/irp6m_const.h"
#include "robot/irp6ot_m/irp6ot_m_const.h"
#include "robot/irp6ot_tfg/irp6ot_tfg_const.h"
#include "robot/irp6p_m/irp6p_m_const.h"
#include "robot/irp6p_tfg/irp6p_tfg_const.h"
#include "robot/polycrank/polycrank_const.h"
#include "robot/smb/smb_const.h"
#include "robot/spkm/spkm_const.h"
#include "robot/shead/shead_const.h"
#include "robot/speaker/speaker_const.h"
#include "robot/conveyor/conveyor_const.h"
#include "robot/bird_hand/bird_hand_const.h"

namespace mrrocpp {
namespace mp {
namespace robot {

class motor_driven : public robot
{
private:
	const int servos_number;

public:
			motor_driven(lib::robot_name_t l_robot_name, const std::string & _section_name, task::task &mp_object_l, int _number_of_servos);
};

} // namespace robot
} // namespace mp
} // namespace mrrocpp
#endif /*MP_R_motor_driven_H_*/
