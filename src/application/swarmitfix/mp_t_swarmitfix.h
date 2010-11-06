// -------------------------------------------------------------------------
//                            task/mp_t_haptic.h
//
// MP task for two robot haptic device
// Ostatnia modyfikacja: 2007
// -------------------------------------------------------------------------

#if !defined(__MP_T_SWARMITFIX_H)
#define __MP_T_SWARMITFIX_H

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
protected:

public:

	swarmitfix(lib::configurator &_config);

	/// utworzenie robotow
	void create_robots(void);
	// methods for mp template
	void main_task_algorithm(void);

};

/** @} */// end of edge_following

} // namespace task
} // namespace mp
} // namespace mrrocpp

#endif
