#if !defined(_ECP_T_TZU_CS_IRP6OT_H)
#define _ECP_T_TZU_CS_IRP6OT_H

#include "ecp/common/ecp_task.h"
#include "ecp/common/ecp_g_smooth.h"
#include "ecp/common/ecp_g_force.h"

#define Z_FORCE_MEASSURE 2
#define X_TORQUE_MEASSURE 3
#define Z_TORQUE_MEASSURE 5
#define NUMBER_OF_TRAJECTORIES 3
#define TRAJECTORY_1 1
#define TRAJECTORY_2 2
#define TRAJECTORY_3 3
#define TRAJECTORY_4 4

class force_meassure_generator;

class ecp_task_tzu_cs_irp6ot :  public ecp_task  
{
protected:
	ecp_smooth_generator *sg;
	bias_edp_force_generator *befg;
	weight_meassure_generator* wmg;
	force_meassure_generator* fmg;
	ecp_force_tool_change_generator* ftcg;
	ecp_tool_change_generator* tcg;
	char* trajectories[NUMBER_OF_TRAJECTORIES];
public:
	ecp_task_tzu_cs_irp6ot(configurator &_config);
	~ecp_task_tzu_cs_irp6ot();
	
	// methods for ECP template to redefine in concrete classes
	/** metoda odpowiedzialna za inicjalizacje zmiennych zadania **/
	void task_initialization(void);
	/** metoda odpowiedzialna za wykonanie zadania **/
	void main_task_algorithm(void);
};

// taki maly prywatny generator
class force_meassure_generator : public ecp_generator
{
private:
    double weight; // pierwszy zmierzony ciezar
    int what_to_meassure;
    
public:
    // konstruktor
    force_meassure_generator(ecp_task& _ecp_task, int what_to_meassure = 2);
	
	void change_meassurement(int what);
	double get_meassurement();
	
    bool first_step ();
    bool next_step ();
}
; // end:


#endif

