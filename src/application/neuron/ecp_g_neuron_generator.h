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

namespace mrrocpp{
namespace ecp{
namespace common{
namespace generator{

/**
 * @brief Generator working with VSP and neural networks.
 * @detials Trajectory generator is intended to work with external VSP module.
 * Generator generates trajectory basing on one coordinates given from VSP for
 * next 5 macro steps, therefor it has to interpolate each macro step between
 * current and given after 5 macro steps. At the beginning it takes first
 * coordinates of a trajectory in first step and later on at the end of 5th
 * macro step and before next 1st macro step current position of a robot is
 * sent to VSP along with other movement assesment data, and VSP returns
 * position for next 5 macro steps, which are processed and interpolated in
 * next step method of the generator.
 */
class neuron_generator: public common::generator::generator{
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
		 * @brief Temporary angle vector used while reading the current robot position.
		 */
		lib::Xyz_Angle_Axis_vector angle_axis_vector;

		/**
		 * @brief Current position of the robot.
		 */
		double actual_position[6];

		/**
		 * @brief Desired position received from vsp.
		 */
		double desired_position[6];

		/**
		 * @brief Array filled with coordinates send to the robot.
		 */
		double position[6];

		/**
		 * @brief Provides information whether start breaking or not.
		 * @details If true, generator tries to break in each axis until robot
		 * stops.
		 */
		bool breaking;

		/**
		 * @brief Current velocity in all axes.
		 */
		double v[6];

		/**
		 * @brief Maximal allowed acceleration in all axes.
		 */
		double a_max[6];

		/**
		 * @brief Node counter used for breaking.
		 * @details When generator knows that it should start breaking, it
		 * calculates how many macro steps its going to need to stop. The
		 * member describes in which breaking macro step according to
		 * calculated one it is.
		 */
		int breaking_node;

		/**
		 * @brief Informs whether desired position is reach or not.
		 * @details Set to true if robot reached desired position for each axes
		 * for currently processed trajectory.
		 */
		bool reached[6];

		/**
		 * @brief Motion direction.
		 */
		int k[6];

		/**
		 * @brief Distance covered in the set of five macrosteps.
		 */
		double s[6];

	public:
		neuron_generator(common::task::task& _ecp_task);
		virtual ~neuron_generator();
		virtual bool first_step();
		virtual bool next_step();

		double * get_position();
		void reset();
};

}//generator
}//common
}//ecp
}//mrrocpp

#endif /* ECP_G_NEURON_GENERATOR_H_ */
