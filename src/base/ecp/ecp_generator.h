/*!
 * \file ecp_generator.h
 * \brief ECP generators base class.
 * - class declaration
 * \author twiniarsk
 * \date 17.03.2008
 */

#if !defined(_ECP_GENERATOR_H)
#define _ECP_GENERATOR_H

#include "base/ecp/ecp_task.h"
#include "base/ecp/ecp_robot.h"
#include "base/ecp_mp/ecp_mp_generator.h"
#include "lib/single_thread_port.h"

namespace mrrocpp {
namespace ecp {
namespace common {
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

	ecp_robot* the_robot;

	generator(common::task::task& _ecp_task);

	virtual ~generator();
};

} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif /* _ECP_GENERATOR_H */
