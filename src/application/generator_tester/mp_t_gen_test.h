#if !defined(__MP_T_GEN_TEST_H)
#define __MP_T_GEN_TEST_H

namespace mrrocpp {
namespace mp {
namespace task {

/**
 * @defgroup gen_test gen_test
 * @ingroup application
 * A gen_test (with active coordinator) QNX test application
 */

class gen_test : public task
{
protected:

public:

	/**
	 * Constructor.
	 */
	gen_test(lib::configurator &_config);
	/// utworzenie robotow
	void create_robots(void);
	void main_task_algorithm(void);

};

} // namespace task
} // namespace mp
} // namespace mrrocpp

#endif
