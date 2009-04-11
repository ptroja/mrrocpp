#if !defined(_ECP_SMOOTH_TAUGHT_IN_POSE_H)
#define  _ECP_SMOOTH_TAUGHT_IN_POSE_H

#include "common/com_buf.h"		// POSE_SPECIFICATION
#include "common/impconst.h"	// MAX_SERVOS_NR

namespace mrrocpp {
namespace ecp {
namespace common {

class ecp_smooth_taught_in_pose {
public:
  POSE_SPECIFICATION arm_type;
  double v_p[MAX_SERVOS_NR];  
  double v_k[MAX_SERVOS_NR];
  double v[MAX_SERVOS_NR];
  double a[MAX_SERVOS_NR];
  double coordinates[MAX_SERVOS_NR];
  
  ecp_smooth_taught_in_pose (void);
  ecp_smooth_taught_in_pose (POSE_SPECIFICATION at, double* vp, double* vk, double* vv, double* aa, double* coordinates);
}; // end:class ecp_smooth_taught_in_pose

} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif /* _ECP_SMOOTH_TAUGHT_IN_POSE_H */
