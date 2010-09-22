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

/*!
 * @brief Base class of all ecp generators
 * The generator both generates command and checks terminal condition
 *
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 * @ingroup ecp
 */
class generator : public ecp_mp::generator::generator
{

protected:
	/**
	 * @brief ecp task object reference
	 */
	common::task::task& ecp_t;

public:
	/**
	 * @brief Main generator method to execute transition cycle
	 */
	void Move(void);

	/**
	 * @brief communicates with EDP
	 */
	virtual void execute_motion(void);

	/**
	 * @brief initiates Move method
	 */
	void move_init(void);

	/**
	 * @brief associated ecp_robot object pointer
	 */
	robot::ecp_robot* the_robot;

	/**
	 * @brief Constructor
	 * @param _ecp_task ecp task object reference.
	 */
	generator(common::task::task& _ecp_task);

	/**
	 * @brief Desstructor
	 */
	virtual ~generator();

	/**
	 * @brief checks robot reply_package and detects edp_error
	 * @return edp_error occurred
	 */
	bool is_EDP_error() const;

	/**
	 * @brief single trajectory node
	 */
	lib::trajectory_description td;

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
