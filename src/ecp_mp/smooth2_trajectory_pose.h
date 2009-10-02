#if !defined(_ECP_SMOOTH2_TRAJECTORY_POSE_H)
#define  _ECP_SMOOTH2_TRAJECTORY_POSE_H

#include "lib/com_buf.h"		// lib::POSE_SPECIFICATION
#include "lib/impconst.h"	// MAX_SERVOS_NR

#include <boost/serialization/utility.hpp>

namespace mrrocpp {
namespace ecp_mp {
namespace common {

class smooth2_trajectory_pose {
public:
  lib::POSE_SPECIFICATION arm_type;
  double v_p[MAX_SERVOS_NR];
  double v_k[MAX_SERVOS_NR];
  double v[MAX_SERVOS_NR];//maksymalna predkosc ruchu dla kazdej osi
  double a[MAX_SERVOS_NR];//maksymalne przyspieszenie ruchu dla kazdej osi
  double coordinates[MAX_SERVOS_NR]; //pozycja docelowa dla kazdej osi

  double przysp[MAX_SERVOS_NR];
  double jedn[MAX_SERVOS_NR];
  double s_jedn[MAX_SERVOS_NR];
  double s_przysp[MAX_SERVOS_NR];

  double start_position[MAX_SERVOS_NR];
  double k[MAX_SERVOS_NR];

  double a_r[MAX_SERVOS_NR];
  double v_r[MAX_SERVOS_NR];
  int interpolation_node_no;
  double v_grip;
  double t; //czas ruchu
  int model[MAX_SERVOS_NR]; //model ruchu
  int pos_num;

  smooth2_trajectory_pose (void);
  smooth2_trajectory_pose (lib::POSE_SPECIFICATION at,
		  const double* coordinates,
		  const double* vv,
		  const double* aa);

private:
	// boost serialization methods
	friend class boost::serialization::access;

	template<class Archive>
	    void serialize(Archive & ar, const unsigned int version) {
		ar & boost::serialization::make_nvp("coordinateType", arm_type);
		ar & boost::serialization::make_nvp("Velocity", v);
		ar & boost::serialization::make_nvp("Accelerations", a);
		ar & boost::serialization::make_nvp("Coordinates", coordinates);
	}
}; // end:class smooth2_trajectory_pose


} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif /* _ECP_SMOOTH2_TRAJECTORY_POSE_H */
