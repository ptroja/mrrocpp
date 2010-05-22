/*!
 * \file ecp_generator.h
 * \brief ECP generators base class.
 * - class declaration
 * \author twiniarsk
 * \date 17.03.2008
 */

#if !defined(_ECP_GENERATOR_H)
#define _ECP_GENERATOR_H

#include "ecp/common/task/ecp_task.h"
#include "ecp/common/ecp_robot.h"
#include "ecp_mp/ecp_mp_generator.h"
#include "lib/single_thread_port.h"


namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {


class generator : public ecp_mp::generator::generator {

	protected:
		common::task::task& ecp_t;

	public:
	    // Zlecenie ruchu dla EDP
  		void Move(void);
  		virtual void execute_motion (void);

  		void move_init (void);


		bool communicate_with_mp_in_move;

		ecp_robot* the_robot;

		generator(common::task::task& _ecp_task);

		virtual ~generator();

		bool is_EDP_error (ecp_robot& _robot) const;
};

} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif /* _ECP_GENERATOR_H */
