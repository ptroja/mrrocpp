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

		bool communicate_with_mp_in_move;
		bool communicate_with_edp;

		ecp_robot* the_robot;

		generator(common::task::task& _ecp_task);

		virtual ~generator();

		bool is_EDP_error (ecp_robot& _robot) const;

		class ECP_error {  // Klasa obslugi bledow generatora
			public:
				const lib::error_class_t error_class;
				const uint64_t error_no;
				lib::edp_error error;

				ECP_error ( lib::error_class_t err_cl, uint64_t err_no, uint64_t err0 = 0, uint64_t err1 = 0);
		}; // end: class ECP_error

};

} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif /* _ECP_GENERATOR_H */
