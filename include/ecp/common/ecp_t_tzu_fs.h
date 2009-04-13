#if !defined(_ECP_T_TZU_FS_H)
#define _ECP_T_TZU_FS_H

#include <iostream>
#include <fstream>

#include "ecp/common/ecp_task.h"
#include "ecp/common/ecp_g_smooth.h"
#include "ecp/common/ecp_g_force.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {
 class force_meassure_generator;
}

namespace task {

#define FORCE_X 0
#define FORCE_Y 1
#define FORCE_Z 2
#define TORQUE_X 3
#define TORQUE_Y 4
#define TORQUE_Z 5
#define TRAJECTORY_VERTICAL_DOWN 0
#define TRAJECTORY_VERTCAL_UP 1
#define TRAJECTORY_HORIZONTAL 2
#define POSTUMENT 0
#define ON_TRACK 1
#define STANDARD 0
#define ALTERNATIVE_X_METHOD_1 1
#define ALTERNATIVE_X_METHOD_2 2
#define ALTERNATIVE_Y_METHOD_1 3
#define ALTERNATIVE_Y_METHOD_2 4
#define NUMBER_OF_TRAJECTORIES 3
#define NUMBER_OF_TEST_TRAJECTORIES 11



class ecp_task_tzu_fs :  public common::task::ecp_task
{
protected:
	generator::ecp_smooth_generator *sg;
	generator::bias_edp_force_generator *befg;
	generator::force_meassure_generator* fmg;
	generator::ecp_force_tool_change_generator* ftcg;
	generator::ecp_tool_change_generator* tcg;
	generator::ecp_tff_nose_run_generator *etnrg;
	const char* trajectories[NUMBER_OF_TRAJECTORIES];
	const char* test_trajectories[NUMBER_OF_TEST_TRAJECTORIES];
	double weight;
	double P_x;
	double P_y;
	double P_z;
	int robot;
	std::ofstream str;
	void set_trajectory(int robot_type, int procedure_type);
	void set_test_trajectory(int robot_type);
	void method_alternative(int type, int sequence[], int T);
	void method_standard(int T);

	const char* get_trajectory(double x[]);
public:
	ecp_task_tzu_fs(configurator &_config);
	~ecp_task_tzu_fs();

	// methods for ECP template to redefine in concrete classes
	/** metoda odpowiedzialna za inicjalizacje zmiennych zadania **/
	void task_initialization(void);
	/** metoda odpowiedzialna za wykonanie zadania **/
	void main_task_algorithm(void);
};
} // namespace task
namespace generator {

// taki maly prywatny generator
class force_meassure_generator : public common::generator::ecp_generator
{
private:
    //double weight;
    int sleep_time;
    int meassurement_count;
    int init_meassurement_count;
	Ft_v_vector weight;
public:
    // konstruktor
    force_meassure_generator(common::task::ecp_task& _ecp_task, int _sleep_time = 0, int _meassurement_count = 1);
	Ft_v_vector& get_meassurement();
	void set_configuration(int _sleep_time, int _meassurement_count);

    bool first_step ();
    bool next_step ();
};

} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp


#endif

