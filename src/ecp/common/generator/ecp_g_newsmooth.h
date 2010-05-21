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

#include "ecp/common/generator/ecp_g_multiple_position.h"
#include "ecp_mp/trajectory_pose/bang_bang_trajectory_pose.h"

using namespace std;

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

/**
 *
 */
class newsmooth : public multiple_position {

	protected:
		/**
		 *
		 */
		bool debug; //czy maja byc wyswietlane debugi
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
		void insert_pose_list_element(lib::ECP_POSE_SPECIFICATION ps, vector<double> v, vector<double> a, vector<double> coordinates);
	public:
		/**
		 *
		 */
		void calculate_interpolate();
		/**
		 * Constructor.
		 */
		newsmooth(common::task::task& _ecp_task, bool _is_synchronised, bool _debug);
		/**
		 * Destructor.
		 */
		virtual ~newsmooth();
		/**
		 *
		 */
		void reset(void);
		/**
		 *
		 */
		void load_a_v_max_from_file(const char* file_name);
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
