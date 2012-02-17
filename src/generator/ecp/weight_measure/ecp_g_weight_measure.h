#if !defined(_ECP_GEN_WEIGHT_MEASURE_H)
#define _ECP_GEN_WEIGHT_MEASURE_H

/*!
 * @file
 * @brief File contains weight measure generator declaration
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * it checks for the weight measurement inside manipulator gripper increase to detect object of certain weight insertion
 * the object to be inserted and detected mass is set
 *
 */
#include "ecp_mp_g_weight_measure.h"
#include "base/ecp/ecp_generator.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

/*!
 * @brief generator to measure weight.
 * The generator do not move robot. First it measures initial force (weight) in x direction of tool.\n
 * Then in the following macrosteps it checks if the filtered (through cyclic buffer) force exceeds the desired increment.\n
 * If it is so it waits the desired time and finishes.\n
 *
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 * @ingroup generators
 */
class weight_measure : public common::generator::generator
{
private:

	/**
	 * @brief cyclic buffer size
	 */
	static const int WEIGHT_MEASSURE_GENERATOR_BUFFER_SIZE = 10;

	/**
	 * @brief single macrostep duration in mikrosecond
	 */
	static const int USLEEP_TIME = 10000;

	/**
	 * @brief weight difference to detect
	 */
	double weight_difference;

	/**
	 * @brief cyclic buffer of measured weights
	 */
	double weight_in_cyclic_buffer[WEIGHT_MEASSURE_GENERATOR_BUFFER_SIZE];

	/**
	 * @brief current pointer of cyclic buffer
	 */
	int current_buffer_pointer;

	/**
	 * @brief first measured weight
	 */
	double initial_weight;

	/**
	 * @brief flag of fist weight measure done
	 */
	bool initial_weight_counted;

	/**
	 * @brief the current number of macrosteps to execute after desired weight difference is detected before the generator is finished
	 */
	int catch_lag;

	/**
	 * @brief the initial number of macrosteps to execute after desired weight difference is detected before the generator is finished
	 */
	int initial_catch_lag;

	/**
	 * @brief time to wait after desired weight difference is detected before the generator is finished
	 */
	double catch_time;

	/**
	 * @brief set if desired weight difference is detected before the generator is finished
	 */
	bool terminate_state_recognized; // wykryto warunek koncowy

	/**
	 * @brief insert measured weight to buffer
	 * @param fx weight
	 */
	void insert_in_buffer(const double fx);

	/**
	 * @brief returns average weight from buffer
	 * @return average weight from buffer
	 */
	double check_average_weight_in_buffer(void) const;

	/**
	 * @brief clears cyclic buffer
	 */
	void clear_buffer();

public:

	/**
	 * @brief sets desired weight difference to detect
	 */
	void set_weight_difference(const double _weight_difference);

	/**
	 * @brief Constructor
	 * @param _ecp_task ecp task object reference.
	 * @param _weight_difference desired weight difference (optional)
	 * @param _catch_time desired catch time to wait after weight difference is detected
	 */
	weight_measure(common::task::task& _ecp_task, double _weight_difference = 0.0, double _catch_time = 1.0);

	/**
	 * @brief generates first step of transition function
	 * @return terminal condition value
	 */
	bool first_step();

	/**
	 * @brief generates next steps (starting from the second) of transition function
	 * @return terminal condition value
	 */
	bool next_step();

	/**
	 * @brief method executed by dispatcher
	 */
	void conditional_execution();

};

} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif
