#if !defined(_ECP_GEN_SPRING_CONTACT_H)
#define _ECP_GEN_SPRING_CONTACT_H

/*!
 * @file
 * @brief File contains ecp_generator class declaration
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup spring_contact
 */
#include "ecp_mp_g_spring_contact.h"
#include "base/ecp/ecp_generator.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

#define MINIMAL_FORCE 1.0
#define FORCE_INCREMENT 2.0
#define POSITION_INCREMENT 0.005
#define HIGH_FORCE_INCREMENT 30.0
#define HIGH_POSITION_INCREMENT 0.015
#define MAX_DIVIDER 4.0
#define STIFFNESS_INSENSIVITY_LEVEL 500.0
#define HIGHEST_STIFFNESS 2000.0

enum HAPTIC_STIFFNESS_STATES
{
	HS_LOW_FORCE = 0, HS_STIFNESS_ESTIMATION
};

/**
 * @brief degrees to radians factor constant
 */
const double DEGREES_TO_RADIANS = 57.295780;

/*!
 * @brief Generator that getting into contact with spring
 *
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 * @ingroup spring_contact
 */
class spring_contact : public common::generator::generator
{
protected:

	/**
	 * @brief number of steps in each macrostep
	 */
	const int step_no;

	/**
	 * @brief manipualtor tool frame
	 */
	lib::Homog_matrix tool_frame;

	HAPTIC_STIFFNESS_STATES irp6p_state;
	double total_irp6p_stiffness;
	double last_irp6p_stiffness;
	double initial_irp6p_force;
	double initial_irp6p_position;
	double intermediate_irp6p_force;
	double intermediate_irp6p_position;

	// Korekta parametrów regulatora siłowego w robocie podrzednym na podstawie estymaty sztywnosci
	double divisor;

public:

	/**
	 * @brief Constructor
	 * @param _ecp_task ecp task object reference.
	 */
	spring_contact(common::task::task& _ecp_task, int step);

	bool first_step();
	bool next_step();

};

} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif
