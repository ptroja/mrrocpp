#ifndef MP_R_MANIP_AND_CONV_H_
#define MP_R_MANIP_AND_CONV_H_

#include "mp/mp.h"
#include "mp/mp_robot.h"
namespace mrrocpp {
namespace mp {
namespace robot {
class manip_and_conv : public robot
{
	private:
		int servos_number;
		bool has_gripper;

	public:
		manip_and_conv (lib::robot_name_t l_robot_name, const char* _section_name, task::task &mp_object_l);

		virtual void create_next_pose_command (void);

		virtual void get_reply (void);
		virtual void get_input_reply (void);
		virtual void get_arm_reply (void);
		virtual void get_rmodel_reply (void);
};
} // namespace robot
} // namespace mp
} // namespace mrrocpp
#endif /*MP_R_MANIP_AND_CONV_H_*/
