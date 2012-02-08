// -------------------------------------------------------------------------
//
// -------------------------------------------------------------------------

#if !defined(_ECP_MP_G_TFF_GRIPPER_APPROACH_H)
#define _ECP_MP_G_TFF_GRIPPER_APPROACH_H

#include "../../../base/lib/com_buf.h"

namespace mrrocpp {
namespace ecp_mp {
namespace generator {
namespace tff_gripper_approach {
enum communication_type
{
	no_data = 0, behaviour_specification = 1
};

class behaviour_specification_data_type
{
public:

	double speed;
	unsigned int motion_time;
	double force_level;

	behaviour_specification_data_type();
	behaviour_specification_data_type(double l_speed, unsigned int l_motion_time, double l_force_level);

	//! Give access to boost::serialization framework
	friend class boost::serialization::access;

	//! Serialization of the data structure
	template <class Archive>
	void serialize(Archive & ar, const unsigned int version)
	{
		// serialize base class information
		ar & speed;
		ar & motion_time;
		ar & force_level;
	}

};

}
const std::string ECP_GEN_TFF_GRIPPER_APPROACH = "ECP_GEN_TFF_GRIPPER_APPROACH";

} // namespace generator
} // namespace ecp_mp
} // namespace mrrocpp

#endif
