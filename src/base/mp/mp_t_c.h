#if !defined(__MP_TASK_C_H)
#define __MP_TASK_C_H

namespace mrrocpp {
namespace mp {
namespace task {

class cxx : public task
{

public:

	cxx(lib::configurator &_config);

	// methods fo mp template to redefine in concete class
	void main_task_algorithm(void);

};

} // namespace task
} // namespace mp
} // namespace mrrocpp

#endif
