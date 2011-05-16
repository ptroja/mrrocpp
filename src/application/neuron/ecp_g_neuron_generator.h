/**
 * @file ecp_g_neuron_generator.h
 * @brief Header file for neuron_generator class
 * @author Tomasz Bem (mebmot@wp.pl)
 * @author Rafal Tulwin (rtulwin@stud.elka.pw.edu.pl)
 * @ingroup neuron
 * @date 02.07.2010
 */

#ifndef ECP_G_NEURON_GENERATOR_H_
#define ECP_G_NEURON_GENERATOR_H_

#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"
#include "neuron_sensor.h"
#include "base/lib/mrmath/mrmath.h"

#include "generator/ecp/ecp_g_teach_in.h"
#include <vector>

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

/**
 * @brief Generator working with VSP and neural networks.
 * @details Trajectory generator is intended to work with external VSP module.
 * Generator generates trajectory basing on one coordinates given from VSP for
 * next 5 macro steps, therefor it has to interpolate each macro step between
 * current and given after 5 macro steps. At the beginning it takes first
 * coordinates of a trajectory in first step and later on at the end of 5th
 * macro step and before next 1st macro step current position of a robot is
 * sent to VSP along with other movement assesment data, and VSP returns
 * position for next 5 macro steps, which are processed and interpolated in
 * next step method of the generator.
 */
class neuron_generator : public common::generator::generator
{
private:

	/**
	 * @brief Communication manager between VSP and MRROC++.
	 * @details Neuron sensor from the point of view of this class is used
	 * to send current position and assesment data and receive next postion
	 */
	ecp_mp::sensor::neuron_sensor *neuron_sensor;

	/**
	 * @brief Matrix to which the current position of the robot is written.
	 */
	lib::Homog_matrix actual_position_matrix;

	/**
	 * @brief Matrix to which the new position for the robot is written.
	 */
	lib::Homog_matrix position_matrix;

	/**
	 * @brief Measured position of the robot.
	 */
	lib::Xyz_Angle_Axis_vector msr_position;

	/**
	 * @brief Desired position received from vsp.
	 */
	double desired_position[6];

	/**
	 * @brief Array filled with coordinates send to the robot.
	 */
	lib::Xyz_Angle_Axis_vector position;

	/**
	 * @brief Provides information whether start breaking or not.
	 * @details If true, generator tries to break in each axis until robot
	 * stops.
	 */
	bool breaking_;

	/**
	 * @brief Time of a macrostep.
	 */
	double t;

	/**
	 * @brief Difference between last and last but one position.
	 * @details Calculated on the VSP side.
	 */
	double normalized_vector[3];

	/**
	 * @brief Value of an overshoot.
	 * @details Overshoot is a maximal distance from the perpendicular hyperplane to
	 * normalized vector.
	 */
	double overshoot_;

	/**
	 * @brief Number of macro steps between consequtive data.
	 */
	uint8_t macroSteps;

	uint8_t mstep_;

	/**
	 * @brief length of breking
	 */
	uint8_t break_steps_;

	/**
	 * @brief Breaking circle radius received from VSP.
	 */
	double radius;

	/**
	 * @brief interpolation polynomial coefficients
	 */
	double coeff_[6][6];

	double vel_[6];

	lib::Xyz_Angle_Axis_vector msr_position_old;
	lib::Xyz_Angle_Axis_vector msr_velocity;

	void velocityProfileLinear(double *coeff, double pos1, double pos2, double t);
	void velocityProfileSpline(double *coeff, double pos1, double vel1, double pos2, double vel2, double time);

public:
	neuron_generator(common::task::task& _ecp_task);
	virtual ~neuron_generator();
	virtual bool first_step();
	virtual bool next_step();

	double get_breaking_time();
	lib::Xyz_Angle_Axis_vector get_position();
	double get_overshoot();
	void reset();
};

}//generator
}//common
}//ecp
}//mrrocpp

#endif /* ECP_G_NEURON_GENERATOR_H_ */
