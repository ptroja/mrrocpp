#ifndef ECP_MP_GENERATOR_H_
#define ECP_MP_GENERATOR_H_

/*!
 * @file
 * @brief File contains ecp_mp base generator declaration
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup ecp_mp
 */

#include "base/ecp_mp/ecp_mp_typedefs.h"

namespace mrrocpp {

namespace lib {
	class sr_ecp;
}

namespace ecp_mp {
namespace generator {

/*!
 * @brief Base class of all ecp and mp generators
 * The generator both generates command and checks terminal condition
 *
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 * @ingroup ecp_mp
 */
class generator
{
private:
	/**
	 * @brief trigger received flag
	 */
	bool trigger;

protected:
	/**
	 * @brief the reference to sr communication object in multi thread version
	 */
	lib::sr_ecp& sr_ecp_msg;

	/**
	 * @brief calls initiate reading method for sensor from sensor map
	 */
	void initiate_sensors_readings();

	/**
	 * @brief calls get reading method for sensor from sensor map
	 */
	void get_sensors_readings();

public:
	/**
	 * @brief Constructor
	 * @param _sr_ecp_msg the reference to sr communication object in multi thread version
	 */
	generator(lib::sr_ecp& _sr_ecp_msg);

	/**
	 * @brief checks if trigger approach and then nulls trigger flag
	 * @return initial trigger flag state
	 */
	bool check_and_null_trigger(); // zwraca wartosc trigger i zeruje go

	/**
	 * @brief sets trigger flag
	 */
	void set_trigger();

	/**
	 * @brief current macrostep number
	 */
	unsigned int node_counter;

	/**
	 * @brief Destructor
	 */
	virtual ~generator();

	/**
	 * @brief the map of sensors
	 */
	sensors_t sensor_m;

	/**
	 * @brief the map of transmitters
	 */
	transmitters_t transmitter_m;

	/**
	 * @brief generates first step of transition function
	 * @return terminal condition value
	 */
	virtual bool first_step(void) = 0;

	/**
	 * @brief generates next steps (starting from the second) of transition function
	 * @return terminal condition value
	 */
	virtual bool next_step(void) = 0;
};

} // namespace generatora
} // namespace ecp_mp
} // namespace mrrocpp

#endif /*ECP_MP_GENERATOR_H_*/
