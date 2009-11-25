// -------------------------------------------------------------------------
//                            ecp_g_smooth2.h dla QNX6
// Deklaracje generatora plynnego ruchu 2 dla ECP
//
// -------------------------------------------------------------------------

#if !defined(_ECP_GEN_SMOOTH2_H)
# define _ECP_GEN_SMOOTH2_H

#include "lib/impconst.h"
#include "lib/com_buf.h"

#include <string.h>

#include <list>
#include "ecp_mp/smooth2_trajectory_pose.h"

#include "ecp/common/ecp_g_jarosz.h"
#include "ecp_mp/Trajectory.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

class smooth2 : public delta {

	protected:
		class coordinates {
			public:
				coordinates(double coordinate[MAX_SERVOS_NR]) {
					memcpy(this->coordinate, coordinate, MAX_SERVOS_NR*sizeof(double));
				};
				double coordinate[MAX_SERVOS_NR];
		};

		std::list<ecp_mp::common::smooth2_trajectory_pose> pose_list;
		std::list<ecp_mp::common::smooth2_trajectory_pose>::iterator pose_list_iterator;

		std::list<coordinates> coordinate_list;
		std::list<coordinates>::iterator coordinate_list_iterator;

		double a_max_motor[MAX_SERVOS_NR], a_max_joint[MAX_SERVOS_NR], a_max_zyz[MAX_SERVOS_NR], a_max_aa[MAX_SERVOS_NR];
		double v_max_motor[MAX_SERVOS_NR], v_max_joint[MAX_SERVOS_NR], v_max_zyz[MAX_SERVOS_NR], v_max_aa[MAX_SERVOS_NR];
		double v_grip_min_zyz, v_grip_min_aa, v_grip_min_motor, v_grip_min_joint;

		int first_interval;
		lib::trajectory_description td;

		bool is_synchronised;
		bool debug; //czy maja byc wyswietlane debugi

		int type; //1 - polozenie bezwzgledne , 2 - polozenie przyrostowe
		bool first_coordinate;
		float distance_eps;

		bool trajectory_generated;
		bool trajectory_calculated;

		bool eq(double a, double b);
		void generate_cords();
		//void calculate_absolute_positions();//for (non real) relative type of given coordinates
		void calculate(void);
		double generate_next_coords(int node_counter, int interpolation_node_no, double start_position, double v_p, double v_r, double v_k, double a_r, int k, double przysp, double jedn, double s_przysp, double s_jedn);

		void reduction_model_1(std::list<ecp_mp::common::smooth2_trajectory_pose>::iterator pose_list_iterator, int i, double s);
		void reduction_model_2(std::list<ecp_mp::common::smooth2_trajectory_pose>::iterator pose_list_iterator, int i, double s);
		void reduction_model_3(std::list<ecp_mp::common::smooth2_trajectory_pose>::iterator pose_list_iterator, int i, double s);
		void reduction_model_4(std::list<ecp_mp::common::smooth2_trajectory_pose>::iterator pose_list_iterator, int i, double s);

		void vk_reduction(std::list<ecp_mp::common::smooth2_trajectory_pose>::iterator pose_list_iterator, int i, double s, double t);
		void vp_reduction(std::list<ecp_mp::common::smooth2_trajectory_pose>::iterator pose_list_iterator, int i, double s, double t);

		void optimize_time1(std::list<ecp_mp::common::smooth2_trajectory_pose>::iterator pose_list_iterator, int i, double s);

		//metody zwiazane z pose_list
		void flush_pose_list(void);
		void initiate_pose_list(void);
		void next_pose_list_ptr(void);
		void prev_pose_list_ptr(void);
		bool is_last_list_element(void);
		void create_pose_list_head(lib::POSE_SPECIFICATION ps, double v[MAX_SERVOS_NR], double a[MAX_SERVOS_NR], double coordinates[MAX_SERVOS_NR]);
		void insert_pose_list_element(lib::POSE_SPECIFICATION ps, double v[MAX_SERVOS_NR], double a[MAX_SERVOS_NR], double coordinates[MAX_SERVOS_NR]);

		//metody zwiazane z coordinate_list
		void flush_coordinate_list(void);
		void initiate_coordinate_list(void);
		int coordinate_list_lenght(void);

	public:
		smooth2(common::task::task& _ecp_task, bool _is_synchronised);
		smooth2(common::task::task& _ecp_task, bool _is_synchronised, bool _debug);

		void set_relative(void); //zmiana na tryb przyrostowy
		void set_absolute(void); //zmiana na tryb bezwzgledny
		void reset(void);

		void load_a_v_max(const char* file_name);
		void load_a_v_min(const char* file_name);
		void load_file_with_path(const char* file_name);
		void load_coordinates(lib::POSE_SPECIFICATION ps, double coordinates[MAX_SERVOS_NR], bool reset);
		void load_coordinates(lib::POSE_SPECIFICATION ps, double v[MAX_SERVOS_NR], double a[MAX_SERVOS_NR], double coordinates[MAX_SERVOS_NR], bool reset);
		void load_coordinates(lib::POSE_SPECIFICATION ps, double cor0, double cor1, double cor2, double cor3, double cor4, double cor5, double cor6, double cor7, bool reset);
		void load_coordinates(lib::POSE_SPECIFICATION ps, double v[MAX_SERVOS_NR], double a[MAX_SERVOS_NR], double cor0, double cor1, double cor2, double cor3, double cor4, double cor5, double cor6, double cor7, bool reset);

		void load_trajectory_from_xml(const char* fileName, const char* nodeName);
		void set_pose_from_xml(xmlNode *stateNode, bool &first_time);
		void load_trajectory_from_xml(ecp_mp::common::Trajectory &trajectory);

		virtual bool first_step();
		virtual bool next_step();

		int pose_list_length(void);
};

} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif
