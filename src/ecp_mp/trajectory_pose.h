#if !defined(_ECP_MP_SMOOTH_TAUGHT_IN_POSE_H)
#define  _ECP_MP_SMOOTH_TAUGHT_IN_POSE_H

#include "lib/com_buf.h"		// lib::POSE_SPECIFICATION
#include "lib/impconst.h"	// MAX_SERVOS_NR

namespace mrrocpp {
namespace ecp_mp {
namespace common {

class trajectory_pose {
public:
  lib::POSE_SPECIFICATION arm_type;
  double v[MAX_SERVOS_NR];
  double a[MAX_SERVOS_NR];
  double coordinates[MAX_SERVOS_NR];

  trajectory_pose (void);
  trajectory_pose (lib::POSE_SPECIFICATION at,
		  const double* coordinates,
		  const double* vv,
		  const double* aa);
};

} // namespace common
} // namespace ecp_mp
} // namespace mrrocpp

#endif /* _ECP_SMOOTH_TAUGHT_IN_POSE_H */
