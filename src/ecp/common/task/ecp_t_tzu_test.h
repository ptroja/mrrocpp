#if !defined(_ECP_T_TZU_TEST_H)
#define _ECP_T_TZU_TEST_H

#include "ecp/common/task/ecp_task.h"
#include "ecp/common/generator/ecp_g_smooth.h"
#include "ecp/common/generator/ecp_g_force.h"

#include <iostream>
#include <fstream>

namespace mrrocpp {
namespace ecp {
namespace common {

namespace generator {
class force_meassure_generator;
} // namespace generator

namespace task {

#define FORCE_X 0
#define FORCE_Y 1
#define FORCE_Z 2
#define TORQUE_X 3
#define TORQUE_Y 4
#define TORQUE_Z 5
#define NOSE 0
#define TEST 1
#define ON_TRACK 0
#define POSTUMENT 1
#define NUMBER_OF_TEST_TRAJECTORIES 11



class tzu_test :  public common::task::task
{
protected:
	generator::smooth *sg;
	generator::bias_edp_force *befg;
	generator::force_meassure_generator* fmg;
	generator::force_tool_change* ftcg;
	generator::tool_change* tcg;
	generator::tff_nose_run *ynrfg;
	const char* test_trajectories[NUMBER_OF_TEST_TRAJECTORIES];
	double weight;
	double P_x;
	double P_y;
	double P_z;
	int procedure_type;
	std::ofstream str;
	int robot;
	void nose_generator_test(int tool);
	void trajectories_test(int count);
	void set_trajectories();
	const char* get_trajectory(double x[]);
	void naciskanie_test();
public:
	tzu_test(lib::configurator &_config);

	/** metoda odpowiedzialna za wykonanie zadania **/
	void main_task_algorithm(void);
};

} // namespace task

namespace generator {

// taki maly prywatny generator
class force_meassure_generator : public common::generator::generator
{
private:
    //double weight;
    int sleep_time;
    int meassurement_count;
    int init_meassurement_count;
	lib::Ft_v_vector weight;
public:
    // konstruktor
    force_meassure_generator(common::task::task& _ecp_task, int _sleep_time = 0, int _meassurement_count = 1);
	lib::Ft_v_vector& get_meassurement();
	void set_configuration(int _sleep_time, int _meassurement_count);

    bool first_step ();
    bool next_step ();
}
; // end:


} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif
