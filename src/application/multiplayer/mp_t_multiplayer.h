#if !defined(__MP_T_MULTIPLAYER_H)
#define _MP_T_MULTIPLAYER_H

namespace mrrocpp {
namespace mp {
namespace task {

class multiplayer : public task
{
private:
	void move_electron_robot(const lib::playerpos_goal_t &goal);

public:
	multiplayer(lib::configurator &_config);
	/// utworzenie robotow
	void create_robots(void);
	// methods for mp template
	void main_task_algorithm(void);
};

} // namespace task
} // namespace mp
} // namespace mrrocpp

#endif
