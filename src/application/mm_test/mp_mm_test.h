#if !defined(__MP_TASK_MM)
#define __MP_TASK_MM

#include "base/mp/mp_task.h"

#include "robot/irp6ot_m/mp_r_irp6ot_m.h"
#include "robot/irp6p_m/mp_r_irp6p_m.h"


#include "robot/irp6ot_tfg/mp_r_irp6ot_tfg.h"
#include "robot/irp6p_tfg/mp_r_irp6p_tfg.h"



#include <vector>

struct Point
{
	int x;
	int y;
};

namespace mrrocpp {
namespace mp {
namespace task {

class mmtest : public task
{

public:


	// konstruktor
	mmtest(lib::configurator &_config);
	// methods for mp template
	void main_task_algorithm(void);
	/// utworzenie robotow
	void create_robots(void);
	void runWaitFunction(int time);
	/*void executeMotion(common::State &state);
	void runEmptyGenForSet(common::State &state);
	void runEmptyGen(common::State &state);
	void runWaitFunction(common::State &state);*/
	void move_down(double mm);
	void move_right(double mm);
	void move_back(double mm);



}; // end : class mp_task_fsautomat

} // namespace task
} // namespace mp
} // namespace mrrocpp

#endif
