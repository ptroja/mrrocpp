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

	public:
		neuron_generator(common::task::task& _ecp_task);
		virtual ~neuron_generator();
		virtual bool first_step();
		virtual bool next_step();

};

}//generator
}//common
}//ecp
}//mrrocpp

#endif /* ECP_G_NEURON_GENERATOR_H_ */
