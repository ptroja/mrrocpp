/*
 * ecp_en_labyrinth.h
 *
 * Author: enatil
 */

#ifndef ECP_EN_LABYRINTH_H
#define ECP_EN_LABYRINTH_H

#include "../../base/ecp/ecp_task.h"
#include "generator/ecp/ecp_g_newsmooth.h"
#include "generator/ecp/ecp_mp_g_newsmooth.h"
#include "g_en_labyrinth.h"
//#include "g_rotate_en_labyrinth.h"
#include "ecp_mp_g_g_en_labyrinth.h"
//#include "ecp_mp_g_g_rotate_en_labyrinth.h"

#include <boost/shared_ptr.hpp>
#include "base/ecp/ecp_task.h"
#include "base/lib/logger.h"
#include "application/visual_servoing/visual_servoing.h"

#include <vector>

#include "base/ecp/ecp_task.h"



struct Point
{
	int x;
	int y;
};



using mrrocpp::ecp::common::generator::single_visual_servo_manager;
using mrrocpp::ecp::common::generator::visual_servo_manager;
using namespace mrrocpp::ecp::servovision;
using boost::shared_ptr;

namespace mrrocpp {
namespace ecp {
namespace common {
namespace task {

class ecp_en_labyrinth: public task {

  protected:
	common::generator::g_en_lab* en_generator;
	common::generator::newsmooth* smooth_generator;

  	shared_ptr<single_visual_servo_manager> sm;
  	shared_ptr<visual_servo> vs;
  	shared_ptr<visual_servo_regulator> reg;
  	shared_ptr<termination_condition> term_cond;

	public:
		ecp_en_labyrinth(lib::configurator &_config);
		//void rotate(double rot,double move, double dir);
		void move_down(double mm);
		void move_right(double mm);
		void move_back(double mm);
		//void main_task_algorithm(void);
		// methods for ECP template to redefine in concrete classes
		void mp_2_ecp_next_state_string_handler(void);
};

}
} // namespace irp6ot
} // namespace ecp
} // namespace mrrocpp


#endif /* ECP_EN_LABYRINTH_H */
