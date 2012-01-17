/*!
 * @file edp_e_spkm.h
 * @brief File containing the declaration of edp::spkm::effector class.
 *
 * @author Tomasz Winiarski
 * @date 2009
 *
 */

#ifndef __EDP_E_SPKM_H
#define __EDP_E_SPKM_H

#include <boost/shared_ptr.hpp>
#include <boost/array.hpp>

#include "base/edp/edp_e_manip.h"
#include "const_spkm.h"
#include "dp_spkm.h"

#include "robot/canopen/gateway.h"
#include "robot/maxon/epos.h"
#include "robot/maxon/ipm_executor.h"

namespace mrrocpp {
namespace edp {
namespace spkm {

/*!
 * @brief class of EDP SwarmItFix parallel kinematic manipulator
 *
 * It is the base of the head mounted on the mobile base.
 */
class effector : public common::manip_effector
{
private:
	/*!
	 * \brief "Desired" joint values that were required by previously received SET command.
	 *
	 *	It is threated as current position of joints - which can be retrieved from the hardware only by the GET command.
	 *
	 */
	lib::JointArray desired_joints_old;

	//! Variable denoting whether current end-effector pose in the cartesian space is known.
	bool is_current_cartesian_pose_known;

	/*!
	 * \brief Tool transformation (SHEAD).
	 * \author tkornuta
	 */
	lib::Homog_matrix shead_frame;

	/*!
	 * \brief Desired tool frame (pose of the SHEAD tip in the PKM base reference frame).
	 * \author tkornuta
	 */
	lib::Homog_matrix desired_spkm_frame;

	/*!
	 * \brief Current tool frame (pose of the SHEAD tip in the PKM base reference frame).
	 * \author tkornuta
	 */
	lib::Homog_matrix current_spkm_frame;

	//! Method checks the state of EPOS controllers.
	void check_controller_state();

protected:
	//! Extension added to both positive and negative limits of every epos controller.
	static const uint32_t limit_extension;

	//! Default axis velocity [rpm]
	uint32_t Vdefault[mrrocpp::lib::spkm::NUM_OF_SERVOS];

	//! Default axis acceleration [rpm/s]
	uint32_t Adefault[mrrocpp::lib::spkm::NUM_OF_SERVOS];

	//! Default axis deceleration [rpm/s]
	uint32_t Ddefault[mrrocpp::lib::spkm::NUM_OF_SERVOS];

	//! Maximal motor velocity [rpm].
	uint32_t MotorVmax[mrrocpp::lib::spkm::NUM_OF_SERVOS];

	//! Max motor acceleration [rpm/s].
	uint32_t MotorAmax[mrrocpp::lib::spkm::NUM_OF_SERVOS];

	//! Access to the CAN gateway unit
	boost::shared_ptr <canopen::gateway> gateway;

	//! PKM axes.
	boost::shared_ptr <maxon::epos> axisA, axisB, axisC, axis1, axis2, axis3;

	//! Axes container.
	boost::array <boost::shared_ptr<maxon::epos>, mrrocpp::lib::spkm::NUM_OF_SERVOS> axes;

	//! Handler for the asynchronous execution of the interpolated profile motion
	maxon::ipm_executor <lib::spkm::NUM_OF_MOTION_SEGMENTS, lib::spkm::NUM_OF_SERVOS> ipm_handler;

public:
	/*!
	 * @brief Constructor.
	 *
	 * The attributes are initialized here.
	 */
	effector(common::shell &_shell, lib::robot_name_t l_robot_name);

	/*!
	 * @brief Destructor.
	 */
	~effector();

	/*!
	 * @brief motors synchronization
	 *
	 * This method synchronizes motors of the robots.
	 */
	void synchronise();

	/*!
	 * @brief Method responsible for synchronization of the MOOG motor.
	 *
	 * Uses velocity and profile position modes instead of the EPOS homing.
	 * @param epos_ controller.
	 * @param velocity_ velocity for seeking of the mechanical stop [rpm].
	 * @param offset_ homing offset [qc].
	 */
	void synchronise_moog_motor(maxon::epos & epos_, int32_t velocity_, int32_t offset_);

	/*!
	 * @brief Disable (thus apply brake) the MOOG motor.
	 */
	void disable_moog_motor();

	/*!
	 * @brief method to create threads other then EDP master thread.
	 *
	 * Here there is only one extra thread - reader_thread.
	 */
	void create_threads();

	/*!
	 * \brief Executes the *move_arm* command.
	 *
	 * It chooses the single thread variant from the motor_driven_effector.
	 *
	 * \param [in] instruction_ - Received command. Parameter UNUSED! due to the fact, that this is a single threaded driver.
	 */
	void move_arm(const lib::c_buffer &instruction_);

	/*!
	 * \brief Method responsible for parsing of the command for motors controlling the legs and SPKM rotation.
	 * \author tkornuta
	 */
	void parse_motor_command();

	/*!
	 * \brief Method responsible for motion of motors controlling the legs and SPKM rotation.
	 * \author tkornuta
	 */
	void execute_motion();

	/*!
	 * \brief Method responsible for interpolated motion in the operational space.
	 * \author tkornuta
	 */
	void interpolated_motion_in_operational_space();

	/*!
	 * \brief Method initializes all SPKM variables (including motors, joints and frames), depending on working mode (robot_test_mode) and robot state.
	 * Called only once after process creation.
	 *
	 * \param [in] instruction_ - Received command. Parameter UNUSED! due to the fact, that this is a single threaded driver.
	 */
	void get_controller_state(lib::c_buffer &instruction_);

	/*!
	 * @brief method to get position of the arm
	 *
	 * \param [in] instruction_ - Received command. Parameter UNUSED! due to the fact, that this is a single threaded driver.
	 */
	void get_arm_position(bool read_hardware, lib::c_buffer &instruction_);

	/*!
	 * @brief method to choose master_order variant
	 *
	 * IHere the single thread variant is chosen
	 */
	void master_order(common::MT_ORDER nm_task, int nm_tryb);

	/*!
	 * \brief method to receive instruction from ecp of particular type
	 */
	lib::INSTRUCTION_TYPE variant_receive_instruction();

	/*!
	 * \brief method to reply to ecp with class of particular type
	 */
	void variant_reply_to_instruction();

	/*!
	 * \brief The particular type of instruction send form ECP to EDP
	 */
	lib::spkm::c_buffer instruction;

	/*!
	 * \brief The particular type of reply send form EDP to ECP
	 */
	lib::spkm::r_buffer reply;
};

} // namespace spkm
} // namespace edp
} // namespace mrrocpp

#endif
