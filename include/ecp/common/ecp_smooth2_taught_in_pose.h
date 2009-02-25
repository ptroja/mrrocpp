#if !defined(_ECP_SMOOTH2_TAUGHT_IN_POSE_H)
#define  _ECP_SMOOTH2_TAUGHT_IN_POSE_H

#include "common/com_buf.h"		// POSE_SPECIFICATION
#include "common/impconst.h"	// MAX_SERVOS_NR

class ecp_smooth2_taught_in_pose {
public:
  POSE_SPECIFICATION arm_type;
  double v[MAX_SERVOS_NR];
  double a[MAX_SERVOS_NR];
  double coordinates[MAX_SERVOS_NR];
  
  ecp_smooth2_taught_in_pose (void);
  ecp_smooth2_taught_in_pose (POSE_SPECIFICATION at, double* vv, double* aa, double* coordinates);
}; // end:class ecp_smooth2_taught_in_pose

#endif /* _ECP_SMOOTH2_TAUGHT_IN_POSE_H */
