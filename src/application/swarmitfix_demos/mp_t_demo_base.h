/*!
 * @file mp_t_smb_powered_from_bench_test.h
 * @brief Class for SMB tests.
 *
 * @date Jan 17, 2012
 * @author tkornuta
 */

#if !defined(__MP_T_SWARMITFIX_DEMO_BASE_H)
#define __MP_T_SWARMITFIX_DEMO_BASE_H


#include "base/mp/mp_task.h"

// MP robot classes.
#include "robot/spkm/mp_r_spkm1.h"
#include "robot/spkm/mp_r_spkm2.h"
#include "robot/smb/mp_r_smb1.h"
#include "robot/smb/mp_r_smb2.h"
#include "robot/shead/mp_r_shead1.h"
#include "robot/shead/mp_r_shead2.h"
#include "robot/sbench/mp_r_sbench.h"

#include "taks_utils.hpp"


namespace mrrocpp {
namespace mp {
namespace task {
namespace swarmitfix {


/** @defgroup swarmitfix swarmitfix
 *  @ingroup application
 *  @{
 */

/*!
 * @brief Base class for all test and demo SwarmItFIX tasks.
 * Contains methods facilitating the control of different type of motions for different agents (SMB, SPKM, SHEAD).
 *
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 * @author tkornuta <tkornuta@ia.pw.edu.pl>, Warsaw University of Technology
 * @date Jan 17, 2012
 */
class demo_base : public mrrocpp::mp::task::task
{

protected:
	//! Name of the SMB robot utilized (SMB1 or SMB2).
	lib::robot_name_t smb_robot_name;

	//! Name of the SPKM robot utilized (SPKM1 or SPKM2).
	lib::robot_name_t spkm_robot_name;

	//! Name of the SHEAD robot utilized (SHEAD1 or SHEAD2).
	lib::robot_name_t shead_robot_name;

	/*!
	 * Moves SMB legs in and out.
	 * @param [in] l1_ Desired position of the leg one (in, out).
	 * @param [in] l2_ Desired position of the leg two (in, out).
	 * @param [in] l3_ Desired position of the leg three (in, out).
	 */
	void move_smb_legs(lib::smb::FESTO_LEG l1_, lib::smb::FESTO_LEG l2_, lib::smb::FESTO_LEG l3_);

	/*!
	 * Sends motor rotation command to SMB in the joint space.
	 * @param [in] legs_rotation_ Desired absolute rotation around leg (in external values -6, -5, ..., 5, 6).
	 * @param [in] pkm_rotation_ Desired absolute rotation of the upper SMP by given angle [radians].
	 */
	void move_smb_external(int legs_rotation_, double pkm_rotation_);

	/*!
	 * @brief Rotates agent around given leg, thus realizes the sequence: pull two legs in, rotate around the third one and pull all legs out.
	 *
	 * @note Desired start as well as final states are 'all legs out'.
	 * @note Utilizes the move_smb_legs() and move_smb_external() methods.
	 * @note PKM rotation is set to zero.
	 *
	 * @param [in] leg_number_ Leg around which the rotation will be performed.
	 * @param [in] rotation_ Rotation of the legs (in external values -6, -5, ..., 5, 6).
	 */
	void rotate_smb(int leg_number_, int rotation_);

	/*!
	 * Controls the head rotation.
	 *
	 * @param [in] joint_ Desired absolute position in the joint space.
	 */
	void move_shead_joints(double joint_);

	/*!
	 * Moves the PKM to the desired position in the joint space
	 *
	 * @param [in] motion_variant_ Variant of the motion to be executed (here only NON_SYNC_TRAPEZOIDAL, SYNC_TRAPEZOIDAL are available).
	 */
	void move_spkm_joints(mrrocpp::lib::epos::EPOS_MOTION_VARIANT motion_variant_, double legA_, double legB_, double legC_, double wrist1_, double wrist2_, double wrist3_);

	/*!
	 * Moves the PKM to the desired pose in the cartesian space.
	 *
	 * @param [in] motion_variant_ Variant of the motion to be executed (here NON_SYNC_TRAPEZOIDAL, SYNC_TRAPEZOIDAL, OPERATIONAL are available).
	 */
	void move_spkm_external(mrrocpp::lib::epos::EPOS_MOTION_VARIANT motion_variant_, double x_, double y_, double z_, double alpha_, double beta_, double gamma_);

	/*!
	 * @brief Method responsible for supporting the plate in give point and return.
	 *  The trajectory is acquired though an intermediate pose (the same intermediate pose is considered in both directions).
	 *
	 * @author tkornuta
	 * @param support_* - xyz_zyz of support pose.
	 * @param inter_* - xyz_zyz of intermediate pose.
	 * @param smb_joint_ - rotation of the SMB (the motor rotating the upper SMB plate).
	 * @param shead_joint - rotation of the SHEAD.
	 */
	void move_to_pose_and_return(
			double support_pkm_x_, double support_pkm_y_, double support_pkm_z_, double support_pkm_alpha_, double support_pkm_beta_, double support_pkm_gamma_,
			double inter_pkm_x_, double inter_pkm_y_, double inter_pkm_z_, double inter_pkm_alpha_, double inter_pkm_beta_, double inter_pkm_gamma_,
			double smb_joint_, double shead_joint_);

	/*!
	 * Controls the bench power supply.
	 */
	void control_bench_power_supply(const mrrocpp::lib::sbench::power_supply_state & ps_, int delay_);

	/*!
	 * Controls the bench cleaning.
	 */
	void control_bench_cleaning(const mrrocpp::lib::sbench::cleaning_state & cs_, int delay_);

	/*!
	 * Controls the bench power (the leg smb is not controlled, thus rotation is simulated).
	 */
	void bench_move_to_power_pose(const bench_pose & pose_, unsigned int delay_);

	/*!
	 * Controls the bench power and cleaning (the leg smb is not controlled, thus rotation is simulated).
	 */
	void bench_move_to_power_pose_with_cleaning(const bench_pose & pose_, unsigned int delay_, unsigned int cleaning_time_);

	/*!
	 * Controls the bench power with rotation of the smb leg.
	 */
	void bench_move_to_power_pose_with_smb(const bench_pose & pose_, unsigned int delay_);

	/*!
	 * Controls the bench power and cleaning with rotation of the smb leg.
	 */
	void bench_move_to_power_pose_with_cleaning_and_smb(const bench_pose & pose_, unsigned int delay_, unsigned int cleaning_time_);


public:
	//! Calls the base class constructor.
	demo_base(lib::configurator &config_);

	//! Empty.
	virtual ~demo_base() { }

};

/** @} */ // end of swarmitfix
} /* namespace swarmitfix */
}// namespace task
} // namespace mp
} // namespace mrrocpp

#endif
