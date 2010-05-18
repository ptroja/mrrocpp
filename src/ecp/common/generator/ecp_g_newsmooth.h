/**
* \file	ecp_g_newsmooth.h
* \brief Smooth class and its methods.
* \author rtulwin
* \date	2009
*
* Smooth trajectory generator having an ability to calculate every trajectory.
*/

#if !defined(_ECP_GEN_NEWSMOOTH_H)
# define _ECP_GEN_NEWSMOOTH_H

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <ctype.h>
#include <iostream>

#include <fstream>
#include <string.h>
#include <vector>

using namespace std;

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"
#include "lib/datastr.h"
#include "lib/srlib.h"
#include "lib/mrmath/mrmath.h"

#include "ecp_mp/bang_bang_trajectory_pose.h"

#include "ecp/common/generator/ecp_g_delta.h"

using namespace std;

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

/**
 *
 */
class newsmooth : public delta {

	protected:
		/**
		 *
		 */
		class coordinates {
			public:
			/**
			 *
			 */
			vector<double> coords;
			/**
			 *
			 */
			coordinates(const vector<double> & coords) {
				this->coords = coords;
			};
		};
		/**
		 *
		 */
		vector<ecp_mp::common::bang_bang_trajectory_pose> pose_vector;
		/**
		 *
		 */
		//vector<ecp_mp::common::bang_bang_trajectory_pose> pose_list_backup;
		/**
		 *
		 */
		vector<ecp_mp::common::bang_bang_trajectory_pose>::iterator pose_vector_iterator;
		/**
		 *
		 */
		//vector<ecp_mp::common::bang_bang_trajectory_pose>::iterator pose_list_backup_iterator;
		/**
		 *
		 */
		vector<coordinates> coordinate_vector;
		/**
		 *
		 */
		vector<coordinates>::iterator coordinate_vector_iterator;
		/**
		 *
		 */
		bool first_interval;
		/**
		 *
		 */
		lib::trajectory_description td;
		/**
		 *
		 */
		bool debug; //czy maja byc wyswietlane debugi
		/**
		 *
		 */
		lib::MOTION_TYPE motion_type;
		/**
		 * Number of axes in which we want to move in the given representation.
		 */
		int axes_num;

		/**
		 *
		 */
		//bool trajectory_generated;
		/**
		 *
		 */
		//bool trajectory_calculated;

		/**
		 *
		 */
		bool eq(double a, double b);
		/**
		 *
		 */
		//void generate_coords();
		/**
		 *
		 */
		//void calculate(void);
		/**
		 *
		 */
		//void send_coordinates(void);
		/**
		 *
		 */
		//double generate_next_coords(int node_counter, int interpolation_node_no, double start_position, double v_p, double v_r, double v_k, double a_r, int k, double przysp, double jedn, double s_przysp, double s_jedn);
		/**
		 *
		 */
		void next_pose_list_ptr(void);
		/**
		 *
		 */
		void prev_pose_list_ptr(void);
		/**
		 *
		 */
		void insert_pose_list_element(lib::ECP_POSE_SPECIFICATION ps, double v[MAX_SERVOS_NR], double a[MAX_SERVOS_NR], double coordinates[MAX_SERVOS_NR]);
		/**
		 *
		 */
		void flush_coordinate_list(void);

	public:
		/**
		 *
		 */
		void set_interpolation_type();
		/**
		 *
		 */
		void calculate_interpolate();
		/**
		 *
		 */
		void set_axes_num(int axes_num);


		/**
		 *
		 */
		newsmooth(common::task::task& _ecp_task, bool _is_synchronised);
		/**
		 *
		 */
		newsmooth(common::task::task& _ecp_task, bool _is_synchronised, bool _debug);
		/**
		 *
		 */
		void set_relative(void); //zmiana na tryb przyrostowy
		/**
		 *
		 */
		void set_absolute(void); //zmiana na tryb bezwzgledny
		/**
		 *
		 */
		void reset(void);
		/**
		 *
		 */
		void load_a_v_max(const char* file_name);


		/**
		 *
		 */
		/*void load_file_with_path(const char* file_name);
		/**
		 *
		 */
		//void load_coordinates(lib::ECP_POSE_SPECIFICATION ps, double coordinates[MAX_SERVOS_NR], bool reset);
		/**
		 *
		 */
		//void load_coordinates(lib::ECP_POSE_SPECIFICATION ps, double v[MAX_SERVOS_NR], double a[MAX_SERVOS_NR], double coordinates[MAX_SERVOS_NR], bool reset);
		/**
		 *
		 */
		//void load_coordinates(lib::ECP_POSE_SPECIFICATION ps, double cor0, double cor1, double cor2, double cor3, double cor4, double cor5, double cor6, double cor7, bool reset);
		/**
		 *
		 */
		//void load_coordinates(lib::ECP_POSE_SPECIFICATION ps, double v[MAX_SERVOS_NR], double a[MAX_SERVOS_NR], double cor0, double cor1, double cor2, double cor3, double cor4, double cor5, double cor6, double cor7, bool reset);
		/**
		 *
		 */
		//void load_xyz_angle_axis(double coordinates[MAX_SERVOS_NR], bool reset);
		/**
		 *
		 */
		//void load_xyz_angle_axis(double v[MAX_SERVOS_NR], double a[MAX_SERVOS_NR], double coordinates[MAX_SERVOS_NR], bool reset);
		/**
		 *
		 */
		//void load_xyz_angle_axis(double cor0, double cor1, double cor2, double cor3, double cor4, double cor5, double cor6, double cor7, bool reset);
		/**
		 *
		 */
		//void load_xyz_angle_axis(double v[MAX_SERVOS_NR], double a[MAX_SERVOS_NR], double cor0, double cor1, double cor2, double cor3, double cor4, double cor5, double cor6, double cor7, bool reset);
		/**
		 *
		 */
		//void load_xyz_euler_zyz(double coordinates[MAX_SERVOS_NR], bool reset);
		/**
		 *
		 */
		//void load_xyz_euler_zyz(double v[MAX_SERVOS_NR], double a[MAX_SERVOS_NR], double coordinates[MAX_SERVOS_NR], bool reset);
		/**
		 *
		 */
		//void load_xyz_euler_zyz(double cor0, double cor1, double cor2, double cor3, double cor4, double cor5, double cor6, double cor7, bool reset);
		/**
		 *
		 */
		//void load_xyz_euler_zyz(double v[MAX_SERVOS_NR], double a[MAX_SERVOS_NR], double cor0, double cor1, double cor2, double cor3, double cor4, double cor5, double cor6, double cor7, bool reset);
		/**
		 *
		 */
		//void load_joint(double coordinates[MAX_SERVOS_NR], bool reset);
		/**
		 *
		 */
		//void load_joint(double v[MAX_SERVOS_NR], double a[MAX_SERVOS_NR], double coordinates[MAX_SERVOS_NR], bool reset);
		/**
		 *
		 */
		//void load_joint(double cor0, double cor1, double cor2, double cor3, double cor4, double cor5, double cor6, double cor7, bool reset);
		/**
		 *
		 */
		//void load_joint(double v[MAX_SERVOS_NR], double a[MAX_SERVOS_NR], double cor0, double cor1, double cor2, double cor3, double cor4, double cor5, double cor6, double cor7, bool reset);
		/**
		 *
		 */
		//void load_motor(double coordinates[MAX_SERVOS_NR], bool reset);
		/**
		 *
		 */
		//void load_motor(double v[MAX_SERVOS_NR], double a[MAX_SERVOS_NR], double coordinates[MAX_SERVOS_NR], bool reset);
		/**
		 *
		 */
		//void load_motor(double cor0, double cor1, double cor2, double cor3, double cor4, double cor5, double cor6, double cor7, bool reset);
		/**
		 *
		 */
		//void load_motor(double v[MAX_SERVOS_NR], double a[MAX_SERVOS_NR], double cor0, double cor1, double cor2, double cor3, double cor4, double cor5, double cor6, double cor7, bool reset);



		/**
		 *
		 */
		virtual bool first_step();
		/**
		 *
		 */
		virtual bool next_step();

};

} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif
