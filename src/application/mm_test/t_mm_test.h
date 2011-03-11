/*
 * t_mm_test.h
 *
 *  Created on: Apr 13, 2010
 *      Author: mmichnie
 */

#ifndef T_MM_TEST_H_
#define T_MM_TEST_H_

#include "../../base/ecp/ecp_task.h"
#include "generator/ecp/ecp_g_newsmooth.h"
#include "g_mm_test.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace task {

class mm_test: public task {

  protected:
	 // common::generator::smooth* smoothgen2;
	common::generator::newsmooth* sg;

	public:
		mm_test(lib::configurator &_config);
		void move_down(double mm);
		void move_right(double mm);
		void move_back(double mm);
		void main_task_algorithm(void);
};

}
} // namespace irp6ot
} // namespace ecp
} // namespace mrrocpp


#endif /* T_MM_TEST_H_ */
