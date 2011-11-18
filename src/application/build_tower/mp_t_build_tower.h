/*
 * mp_t_build_tower.h
 *
 *  Created on: 17-11-2011
 *      Author: spiatek
 */

#ifndef MP_T_BUILD_TOWER_H_
#define MP_T_BUILD_TOWER_H_

namespace mrrocpp {
namespace mp {
namespace task {

class build_tower : public task
{
protected:

public:

	build_tower(lib::configurator &_config);

	void create_robots(void);
	void main_task_algorithm(void);
};

} // namespace task
} // namespace mp
} // namespace mrrocpp

#endif /* MP_T_BUILD_TOWER_H_ */
