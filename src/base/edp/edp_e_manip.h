/*!
 * \file edp_e_manip.h
 * \brief File containing the declaration of edp::common::motor_driven_effector class.
 *
 * \author yoyek
 * \date 2009
 *
 */

#ifndef __EDP_E_MANIP_H
#define __EDP_E_MANIP_H

#include "base/edp/edp_e_motor_driven.h"

// Konfigurator
#include "base/lib/configurator.h"

namespace mrrocpp {
namespace edp {
namespace sensor {
class force;
}
namespace common {

//
/*!
 * \brief base class for EDP robotic manipulators
 *
 * It extends motor_driven_effector by the methods of computation of the end--effector position with direct and inverse kinematic task
 * and some methods of govering data for the purpose of position--force control.
 */
class manip_effector : public common::motor_driven_effector
{

protected:

	/*!
	 * \brief method that computes desired_motor_pos_new basing on the end-effector frame commanded by the ECP
	 *
	 * It can be reimplemented in the inherited classes
	 */
	virtual void compute_frame(const lib::c_buffer &instruction);

	/*!
	 * \brief the matrix of the end effector frame without tool for the servo buffer (pose of the WRIST).
	 *
	 * It is computed for every single step of the motion.
	 */
	lib::Homog_matrix servo_current_frame_wo_tool;

	/*!
	 * \brief desired end--effector frame
	 *
	 * for the whole macrostep
	 */
	lib::Homog_matrix desired_end_effector_frame;

	/*!
	 * \brief current end--effector frame
	 *
	 * set once for the macrostep execution
	 */
	lib::Homog_matrix current_end_effector_frame;

	/*!
	 * \brief current global force measurement.
	 *
	 * It is set by the force thread.
	 */
	lib::Ft_vector global_force_msr;

	/*!
	 * \brief mutex for force global_force_msr
	 *
	 * This measueremnt is set by the force thread and get by the transformation thread.
	 */
	boost::mutex force_mutex; // mutex do sily   XXXXXX

	/*!
	 * \brief move arm method for the FRAME command in the single thread variant.
	 *
	 * It also calls single_thread_move_arm method from the motor_drive_effector
	 */
	void single_thread_move_arm(const lib::c_buffer &instruction);

	/*!
	 * \brief move arm method for the FRAME command in the two thread variant.
	 *
	 * It also calls multi_thread_move_arm method from the motor_drive_effector
	 */
	void multi_thread_move_arm(const lib::c_buffer &instruction);

public:

	/*!
	 * \brief Info if the force sensor test mode is active.
	 *
	 * It is taken from configuration data.
	 */
	bool force_sensor_test_mode;

	/*!
	 * \brief geat_arm_position methos with servo_buffer and force measurement
	 *
	 * One of the variant of get_arm_position method commonly choosen in inherited robots.
	 */
	void get_arm_position_with_force_and_sb(bool, lib::c_buffer &);

	/*!
	 * \brief method to set the robot model commanded by ECP with usage of ser_buffer
	 *
	 * This method extends set_robot_model_method in case of usage of servo_buffer thread
	 */
	void set_robot_model_with_sb(const lib::c_buffer &instruction);

	/*!
	 * \brief manip_effector class constructor
	 *
	 * The attributes are initialized here.
	 */
	manip_effector(lib::configurator &_config, lib::robot_name_t l_robot_name);

	/*!
	 * \brief methods returns servo_current_frame_wo_tool
	 *
	 * there are two variants just the servo_current_frame_wo_tool frame and the same frame with removed translation
	 */
	lib::Homog_matrix return_current_frame(TRANSLATION_ENUM translation_mode);

	/*!
	 * \brief computation of the base_pos_xyz_rot_xyz_vector.
	 *
	 * For the purpose of the position-force control. It is called once from the pose_force_torque_at_frame_move.
	 */
	void
			compute_base_pos_xyz_rot_xyz_vector(const lib::JointArray & begining_joints, const lib::Homog_matrix & begining_end_effector_frame, const lib::c_buffer & instruction, lib::Xyz_Angle_Axis_vector & base_pos_xyz_rot_xyz_vector);

	/*!
	 * \brief Iteration (interpolation) of the position-force control motion.
	 *
	 * It bases on the pose_force_torque_at_frame_move and other ECP command arguments.
	 */
	virtual void
			iterate_macrostep(const lib::JointArray & begining_joints, const lib::Homog_matrix & begining_end_effector_frame, const lib::c_buffer & instruction, const lib::Xyz_Angle_Axis_vector & base_pos_xyz_rot_xyz_vector);

	/*!
	 * \brief pose-force command execution
	 *
	 * The main function of the pose-force command execution (interpolation)
	 * It calls compute_base_pos_xyz_rot_xyz_vector and iterate_macrostep methods.
	 */
	void pose_force_torque_at_frame_move(const lib::c_buffer &instruction);

	/*!
	 * \brief method to set global_force_msr with mutex protection.
	 *
	 * It is called in the force sensor thread.
	 */
	void force_msr_upload(const lib::Ft_vector & l_vector);

	/*!
	 * \brief method to get global_force_msr with mutex protection.
	 *
	 * It is called in the transformation thread.
	 */
	void force_msr_download(lib::Ft_vector & l_vector);

	/*!
	 * \brief method that computes servo_current_frame_wo_tool
	 *
	 * It also call servo_current_frame_wo_tool method of the motor_driven_effector class
	 */
	virtual bool compute_servo_joints_and_frame(void);

	/*!
	 * \brief motors synchronisation
	 *
	 * This method synchronises motors of the robots.
	 */
	void synchronise();

	/*!
	 * \brief The method checks the initial state of the controller.
	 *
	 * This method typically communicates with hardware to check if the robot is synchronised etc.
	 */
	void get_controller_state(lib::c_buffer &instruction);

	/*!
	 * \brief method to set the robot model commanded by ECP
	 *
	 * The model consists of tool_frame and models handled in set_robot_model method of motor_driven_effector called here
	 */
	virtual void set_robot_model(const lib::c_buffer &instruction);

	/*!
	 * \brief method to get (read) the robot model
	 *
	 * The model consists of tool_frame and models handled in set_robot_model method of motor_driven_effector called here.
	 * Then it is sent to the ECP.
	 */
	virtual void get_robot_model(lib::c_buffer &instruction);

	/*!
	 * \brief commonly used part of the get_arm_position method
	 *
	 * it defines the execution for the frame coordinates and
	 * calls the get_arm_position_get_arm_type_switch method of the motor_driven_effector class.
	 */
	virtual void get_arm_position_get_arm_type_switch(lib::c_buffer &instruction);

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} // namespace common
} // namespace edp
} // namespace mrrocpp

#endif
