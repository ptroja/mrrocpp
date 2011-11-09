#if !defined(__MP_T_BLOCK_MOVE_H)
#define __MP_T_BLOCK_MOVE_H

namespace mrrocpp {
namespace mp {
namespace task {

class block_move : public task
{
protected:

public:

	block_move(lib::configurator &_config);

	void create_robots(void);
	void main_task_algorithm(void);
/*
private:

	const std::string config_section_name;
	int vs_settle_time;
*/
};

} // namespace task
} // namespace mp
} // namespace mrrocpp

#endif
