#if !defined(_ECP_SMOOTH2_TAUGHT_IN_POSE_H)
#define  _ECP_SMOOTH2_TAUGHT_IN_POSE_H

#include "common/com_buf.h"		// POSE_SPECIFICATION
#include "common/impconst.h"	// MAX_SERVOS_NR

class ecp_smooth2_taught_in_pose {
public:
  POSE_SPECIFICATION arm_type;
  double v[MAX_SERVOS_NR];//maksymalna predkosc ruchu dla kazdej osi
  double a[MAX_SERVOS_NR];//maksymalne przyspieszenie ruchu dla kazdej osi
  double coordinates[MAX_SERVOS_NR]; //pozycja docelowa dla kazdej osi

  double przysp[MAX_SERVOS_NR];
  double jedn[MAX_SERVOS_NR];
  double s_jedn[MAX_SERVOS_NR];
  double s_przysp[MAX_SERVOS_NR];

  double start_position[MAX_SERVOS_NR];
  double k[MAX_SERVOS_NR];
  double v_p[MAX_SERVOS_NR];
  double v_k[MAX_SERVOS_NR];
  double a_r[MAX_SERVOS_NR];
  double v_r[MAX_SERVOS_NR];
  int interpolation_node_no;
  double v_grip;
  double t;

  ecp_smooth2_taught_in_pose (void);
  ecp_smooth2_taught_in_pose (POSE_SPECIFICATION at, double* vv, double* aa, double* coordinates);
}; // end:class ecp_smooth2_taught_in_pose

#endif /* _ECP_SMOOTH2_TAUGHT_IN_POSE_H */
