/*
 * ecp_g_neuron_generator.h
 *
 *  Created on: Jul 2, 2010
 *      Author: tbem
 */

#ifndef ECP_G_NEURON_GENERATOR_H_
#define ECP_G_NEURON_GENERATOR_H_

#include "lib/impconst.h"
#include "lib/com_buf.h"
#include "neuron_sensor.h"
#include "lib/mrmath/mrmath.h"

#include "generator/ecp/ecp_g_teach_in.h"
#include <vector>

namespace mrrocpp{
namespace ecp{
namespace common{
namespace generator{

class neuron_generator: public common::generator::generator{
	private:
		ecp_mp::sensor::neuron_sensor *neuron_sensor;

		lib::Homog_matrix actual_position_matrix;
		//lib::Homog_matrix desired_position_matrix;
		lib::Xyz_Angle_Axis_vector angle_axis_vector;
		double actual_position[6];
		double desired_position[6];
		lib::Homog_matrix position_matrix;
		double position[6];
		/**
		 * Vector filled with coordinates read from the robot.
		 */
		//vector<double> position;
		/**
		 * If true, generator tries to break in each axis until robot stops.
		 */
		bool breaking;
		/**
		 * Current velocity in all axes.
		 */
		double v[6];
		/**
		 * Maximal allowed acceleration in all axes.
		 */
		double a_max[6];
		/**
		 * Node counter used for breaking.
		 */
		int breaking_node;
		/**
		 * Set to true if robot reached desired position for the whole trajectory in each axis.
		 */
		bool reached[6];
		/**
		 * Motion direction.
		 */
		double k[6]; // motion direction
		/**
		 * Distance covered in the set of five macrosteps.
		 */
		double s[6];

	public:
		neuron_generator(common::task::task& _ecp_task);
		virtual ~neuron_generator();
		virtual bool first_step();
		virtual bool next_step();
		/**
		 * Returns current robot position.
		 */
		double * get_position();
		void reset();
		void set_breaking(bool breaking);
};

}//generator
}//common
}//ecp
}//mrrocpp

#endif /* ECP_G_NEURON_GENERATOR_H_ */
