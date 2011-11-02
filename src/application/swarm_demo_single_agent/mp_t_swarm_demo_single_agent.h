#if !defined(__MP_T_SWARM_DEMO_SIGLE_AGENT_H)
#define __MP_T_SWARM_DEMO_SIGLE_AGENT_H

#include "robot/spkm/mp_r_spkm2.h"
#include "robot/smb/mp_r_smb2.h"

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

	void move_smb(int leg_number, double rotation);

	void move_smb_external(double x1, double x2);

	void move_spkm_joints(double x1, double x2, double x3, double x4, double x5, double x6);
};

/** @} */ // end of swarmitfix
}// namespace task
} // namespace mp
} // namespace mrrocpp

#endif
