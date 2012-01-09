#if !defined(__MP_T_BLOCK_MOVE_H)
#define __MP_T_BLOCK_MOVE_H

#include <list>
#include <string>

#include <gecode/int.hh>

#include "BlockPlanner.h"
#include "BlockPosition.h"

typedef list<BlockPosition> block_position_list;

namespace mrrocpp {
namespace mp {
namespace task {

class block_move : public task
{

protected:

	int present_color;
	std::vector <int> present_position;

	block_position_list planned_list;

public:

	block_position_list get_list_from_file(const char*);
	block_position_list create_plan(block_position_list);

	block_move(lib::configurator &_config);

	void create_robots(void);
	void main_task_algorithm(void);

};

} // namespace task
} // namespace mp
} // namespace mrrocpp

#endif
