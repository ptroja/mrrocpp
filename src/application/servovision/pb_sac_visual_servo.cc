/*
 * pb_sac_visual_servo.cc
 *
 *  Created on: May 19, 2010
 *      Author: mboryn
 */

#include <iostream>

#include "pb_sac_visual_servo.h"

using ecp_mp::sensor::fradia_sensor;
using namespace logger;
using namespace visual_servo_types;
using namespace std;

namespace mrrocpp {

namespace ecp {

namespace servovision {

pb_sac_visual_servo::pb_sac_visual_servo(boost::shared_ptr <visual_servo_regulator> regulator, const std::string& section_name, mrrocpp::lib::configurator& configurator) :
	pb_visual_servo(regulator, section_name, configurator)
{
	O_T_C = configurator.value <3, 4> ("O_T_C", section_name);
	//	cout << "\nO_T_C\n" << O_T_C << endl;
}

pb_sac_visual_servo::~pb_sac_visual_servo()
{
}

lib::Homog_matrix pb_sac_visual_servo::compute_position_change(const lib::Homog_matrix& current_position, double dt)
{
	lib::Homog_matrix C_T_G(vsp_fradia->get_reading_message().position);
	lib::Homog_matrix error_matrix;
	lib::Homog_matrix E_T_O = !current_position;

//	{
//		cout << "\nC_T_G:\n" << C_T_G << endl;
//
//		lib::Homog_matrix O_T_G = O_T_C * C_T_G;
//		cout << "\nO_T_G:\n" << O_T_G << endl;
//
//		cout << "\ncurrent_position:\n" << current_position << endl;
//		cout << "\nE_T_O:\n" << E_T_O << endl;
//
//		lib::Homog_matrix E_T_C = E_T_O * O_T_C;
//		cout << "\nE_T_O * O_T_C:\n" << E_T_C << endl;
//
//		lib::Homog_matrix E_T_G = E_T_O * O_T_C * C_T_G;
//		cout << "\nE_T_O * O_T_C * C_T_G:\n" << E_T_G << endl;
//	}

	error_matrix = G_T_E_desired * E_T_O * O_T_C * C_T_G;

	//error_matrix = E_T_O * O_T_C * C_T_G;

//	cout << "\nerror_matrix:\n" << error_matrix << endl;
//	cout.flush();

	lib::Xyz_Angle_Axis_vector aa_vector;
	error_matrix.get_xyz_angle_axis(aa_vector);
	error = aa_vector;

//	log_dbg("aa_vector: %10lg, %10lg, %10lg, %10lg, %10lg, %10lg\n", aa_vector(0, 0), aa_vector(1, 0), aa_vector(2, 0), aa_vector(3, 0), aa_vector(4, 0), aa_vector(5, 0));

	aa_vector = regulator->compute_control(aa_vector, dt);
//	log_dbg("aa_vector after regulation: %10lg, %10lg, %10lg, %10lg, %10lg, %10lg\n", aa_vector(0, 0), aa_vector(1, 0), aa_vector(2, 0), aa_vector(3, 0), aa_vector(4, 0), aa_vector(5, 0));

	lib::Homog_matrix delta_position;
	delta_position.set_from_xyz_angle_axis(aa_vector);
	return delta_position;
}

}//namespace

}

}

