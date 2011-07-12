/**
 * @file mp_t_stats.h
 * @brief Header file for stats class.
 * @author Tomasz Bem (mebmot@wp.pl)
 * @ingroup stats
 * @date 25.06.2010
 */

#ifndef MP_T_STATS_H_
#define MP_T_STATS_H_

namespace mrrocpp {
namespace mp {
namespace task {

/**
 * @brief stats task class declaration.
 * @details Taks for generation statistics for a given trajectory.
 */
class stats : public task
{
public:
	stats(lib::configurator &_config);
	void main_task_algorithm(void);
	/// utworzenie robotow
	void create_robots(void);
	virtual ~stats();
};

} //task
} //mp
} //mrrocpp


#endif /* MP_T_STATS_H_ */
