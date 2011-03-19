// -------------------------------------------------------------------------
//                            mp_t_haptic_stiffness.h
//
// MP task for two robot haptic_stiffness device
//
// -------------------------------------------------------------------------

#if !defined(__MP_T_HAPTIC_STIFFNESS_H)
#define __MP_T_HAPTIC_STIFFNESS_H

namespace mrrocpp {
namespace mp {
namespace task {

/** @defgroup haptic_stiffness Haptic coupling
 *  @ingroup application
 *  A two robot application of IRp6 manipulator's in haptic_stiffness coupling
 *  @{
 */

class haptic_stiffness : public task
{
protected:

	void configure_edp_force_sensor(bool configure_track, bool configure_postument);

public:

	haptic_stiffness(lib::configurator &_config);
	/// utworzenie robotow
	void create_robots(void);
	// methods for mp template
	void main_task_algorithm(void);

};

/** @} */

} // namespace task
} // namespace mp
} // namespace mrrocpp

#endif
