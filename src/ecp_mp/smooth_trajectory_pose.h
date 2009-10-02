#if !defined(_ECP_MP_SMOOTH_TRAJECTORY_POSE_H)
#define  _ECP_MP_SMOOTH_TRAJECTORY_POSE_H

#include "lib/com_buf.h"	// lib::POSE_SPECIFICATION
#include "lib/impconst.h"	// MAX_SERVOS_NR

#include <boost/serialization/utility.hpp>

namespace mrrocpp {
namespace ecp_mp {
namespace common {

class smooth_trajectory_pose {
	public:
	  lib::POSE_SPECIFICATION arm_type;
	  double v_p[MAX_SERVOS_NR];
	  double v_k[MAX_SERVOS_NR];
	  double v[MAX_SERVOS_NR];
	  double a[MAX_SERVOS_NR];
	  double coordinates[MAX_SERVOS_NR];

	  smooth_trajectory_pose (void);
	  smooth_trajectory_pose (lib::POSE_SPECIFICATION at,
			  const double* coordinates,
			  const double* vv,
			  const double* aa,
			  const double* vp,
			  const double* vk);

	private:
		// boost serialization methods
		friend class boost::serialization::access;

		template<class Archive>
		    void serialize(Archive & ar, const unsigned int version) {
			ar & boost::serialization::make_nvp("coordinateType", arm_type);
			ar & boost::serialization::make_nvp("StartVelocity", v_p);
			ar & boost::serialization::make_nvp("EndVelocity", v_k);
			ar & boost::serialization::make_nvp("Velocity", v);
			ar & boost::serialization::make_nvp("Accelerations", a);
			ar & boost::serialization::make_nvp("Coordinates", coordinates);
		}
};

} // namespace common
} // namespace ecp_mp
} // namespace mrrocpp

#endif /* _ECP_SMOOTH_TRAJECTORY_POSE_H */
