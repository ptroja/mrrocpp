#if !defined(_ECP_GENERATOR_H)
#define _ECP_GENERATOR_H

/*!
 * @file
 * @brief File contains ecp base generator declaration
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup ecp
 */

#include "base/ecp_mp/ecp_mp_generator.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace robot {
class ecp_robot;
}

namespace task {
class task;
} // namespace task


namespace generator {

class generator : public ecp_mp::generator::generator
{

protected:
	common::task::task& ecp_t;

public:
	// Zlecenie ruchu dla EDP
	void Move(void);
	virtual void execute_motion(void);

	void move_init(void);

	bool communicate_with_mp_in_move;

	robot::ecp_robot* the_robot;

	generator(common::task::task& _ecp_task);

	virtual ~generator();

	bool is_EDP_error(robot::ecp_robot& _robot) const;

	lib::trajectory_description td;

	//virtual bool first_step () = 0;
	//virtual bool next_step () = 0;

};

/*!
 * @brief ECP generator error handling class
 *
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 * @ingroup ecp
 */
class ECP_error
{
public:
	/**
	 * @brief error class (type)
	 */
	const lib::error_class_t error_class;

	/**
	 * @brief error number
	 */
	const uint64_t error_no;

	/**
	 * @brief edp error structure
	 */
	lib::edp_error error;

	/**
	 * @brief constructor
	 * @param err_cl error class
	 * @param err_no error number
	 * @param err0 EDP error0 number
	 * @param err1 EDP error1 number
	 */
	ECP_error(lib::error_class_t err_cl, uint64_t err_no, uint64_t err0 = 0, uint64_t err1 = 0);
};

} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif /* _ECP_GENERATOR_H */
