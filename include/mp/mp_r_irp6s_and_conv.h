#ifndef MP_R_IRP6S_AND_CONV_H_
#define MP_R_IRP6S_AND_CONV_H_

#include "mp/mp_robot.h"

class mp_irp6s_and_conv_robot : public mp_robot
{
	private:
		int servos_number;
		bool has_gripper;

	public:
		mp_irp6s_and_conv_robot (ROBOT_ENUM l_robot_name, const char* _section_name, mp_task &mp_object_l);

		virtual void create_next_pose_command (void);

		virtual void get_reply (void);
		virtual void get_input_reply (void);
		virtual void get_arm_reply (void);
		virtual void get_rmodel_reply (void);
};

#endif /*MP_R_IRP6S_AND_CONV_H_*/
