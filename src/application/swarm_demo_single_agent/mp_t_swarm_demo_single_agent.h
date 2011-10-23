#if !defined(__MP_T_SWARM_DEMO_SIGLE_AGENT_H)
#define __MP_T_SWARM_DEMO_SIGLE_AGENT_H

namespace mrrocpp {
namespace mp {
namespace task {

/** @defgroup swarmitfix swarmitfix
 *  @ingroup application
 *  A swarmitfix QNX test application
 *  @{
 */

class swarmitfix : public task
{
public:
	swarmitfix(lib::configurator &_config);

	/// utworzenie robotow
	void create_robots(void);

	// methods for mp template
	void main_task_algorithm(void);
};

/** @} */ // end of swarmitfix
}// namespace task
} // namespace mp
} // namespace mrrocpp

#endif
