/*
 * mp_t_build_tower.h
 *
 *  Created on: 17-11-2011
 *      Author: spiatek
 */

#ifndef MP_T_BUILD_TOWER_H_
#define MP_T_BUILD_TOWER_H_

#include <list>
#include <string>

#include "BlockPosition.h"

typedef list<BlockPosition> block_position_list;

namespace mrrocpp {
namespace mp {
namespace task {

class build_tower : public task
{

protected:

	const char* present_color;
	std::vector <int> present_position;

	block_position_list planned_list;

public:

	block_position_list get_list_from_file(const char*);
	block_position_list create_plan(block_position_list);

	build_tower(lib::configurator &_config);

	void create_robots(void);
	void main_task_algorithm(void);

};

} // namespace task
} // namespace mp
} // namespace mrrocpp

#endif /* MP_T_BUILD_TOWER_H_ */
