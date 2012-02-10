/*
 * generator/ecp_g_en_labyrinth.h
 *
 *Author: Emil Natil
 */

#ifndef ECP_G_EN_LABYRINTH_H_
#define ECP_G_EN_LABYRINTH_H_

#include <ctime>

#include "base/ecp/ecp_generator.h"
#include "generator/ecp/newsmooth/ecp_g_newsmooth.h"
#include "ecp_mp_g_en_labyrinth.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

class en_labyrinth : public common::generator::newsmooth
{
public:
	en_labyrinth(common::task::task& _ecp_task); //constructor

	void conditional_execution();
};

} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif /* ECP_G_SLEEP_H_ */
