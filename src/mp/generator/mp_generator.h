#ifndef MP_GENERATOR_H_
#define MP_GENERATOR_H_

#include "mp/robot/mp_robot.h"
#include "ecp_mp/ecp_mp_generator.h"

namespace mrrocpp {
namespace mp {
namespace generator {

class generator : public ecp_mp::generator::generator
{
	private:
		// Zadanie, któremu podlega generator
		task::task& mp_t;

	public:
		// Funkcja ruchu
		void Move (void);

		//! mapa wszystkich robotow
		common::robots_t robot_m;

		//! Konstruktor
		generator(task::task& _mp_task);
};

} // namespace common
} // namespace mp
} // namespace mrrocpp

#endif /*MP_GENERATOR_H_*/
