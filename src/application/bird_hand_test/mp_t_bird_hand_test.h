// -------------------------------------------------------------------------
//                            task/mp_t_haptic.h
//
// MP task for two robot haptic device
// Ostatnia modyfikacja: 2007
// -------------------------------------------------------------------------

#if !defined(__MP_T_BIRD_HAND_TEST_H)
#define __MP_T_BIRD_HAND_TEST_H


namespace mrrocpp {
namespace mp {
namespace task {

/** @defgroup bird_hand_test bird_hand_test
 *  @ingroup application
 *  A bird_hand_test QNX test application
 *  @{
 */

class bird_hand_test: public task {
protected:

public:

	bird_hand_test(lib::configurator &_config);
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
