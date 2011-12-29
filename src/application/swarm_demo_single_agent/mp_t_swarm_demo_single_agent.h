#if !defined(__MP_T_SWARM_DEMO_SIGLE_AGENT_H)
#define __MP_T_SWARM_DEMO_SIGLE_AGENT_H

#include "robot/spkm/mp_r_spkm1.h"
#include "robot/spkm/mp_r_spkm2.h"
#include "robot/smb/mp_r_smb1.h"
#include "robot/smb/mp_r_smb2.h"
#include "robot/shead/mp_r_shead1.h"
#include "robot/shead/mp_r_shead2.h"

namespace mrrocpp {
namespace mp {
namespace task {

/** @defgroup swarmitfix swarmitfix
 *  @ingroup application
 *  A swarmitfix QNX test application
 *  @{
 */

class swarmitfix : public task
{
public:
	swarmitfix(lib::configurator &_config);

	/// utworzenie robotow
	void create_robots(void);

	// methods for mp template
	void main_task_algorithm(void);

	void move_smb_legs(lib::smb::FESTO_LEG l1, lib::smb::FESTO_LEG l2, lib::smb::FESTO_LEG l3);

	void rotate_smb(int leg_number, double rotation);

	void move_smb_and_spkm(int leg_number, double rotation);

	void move_smb_external(double x1, double x2);

	void move_spkm_joints(double x1, double x2, double x3, double x4, double x5, double x6);
	void move_shead_joints(double x1);
	void move_spkm_external(mrrocpp::lib::epos::EPOS_MOTION_VARIANT motion_variant_, double x1, double x2, double x3, double x4, double x5, double x6);

	/*!
	 * \brief Method responsible for supporting the plate in give point and return.
	 *  The trajectory is acquired though and intermediate pose (the same intermediate pose is considered in both directions).
	 *  \author tkornuta
	 *  \param support_* - xyz_zyz of support pose.
	 *  \param inter_* - xyz_zyz of intermediate pose.
	 *  \param smb_joint_ - rotation of the SMB (the motor rotating the upper SMB plate).
	 *  \param shead_joint - rotation of the SHEAD.
	 */
	void move_to_pose_and_return(
			double support_pkm_x_, double support_pkm_y_, double support_pkm_z_, double support_pkm_alpha_, double support_pkm_beta_, double support_pkm_gamma_,
			double inter_pkm_x_, double inter_pkm_y_, double inter_pkm_z_, double inter_pkm_alpha_, double inter_pkm_beta_, double inter_pkm_gamma_,
			double smb_joint_, double shead_joint_);

};

/** @} */ // end of swarmitfix
}// namespace task
} // namespace mp
} // namespace mrrocpp

#endif
