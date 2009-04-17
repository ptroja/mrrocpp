// ecp_g_visioncoordinates.cc - generator odpowiadajacy za obliczanie wspolrzednych dla ruchu w strone
//      obiektu zaobserowanego przez system wizyjny (pobiera dane z VSP FraDIA)
// (c)2009 Maciej Jerzy Nowak
// Created: 20.02.2009  Last Update: 20.02.2009
///////////////////////////////////////////////////////////////////////////////

//#include <stdio.h>
//#include <fstream>
#include <iostream>
#include <sstream>
//#include <time.h>
//#include <unistd.h>

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "lib/srlib.h"
#include "lib/mathtr.h"

#include "ecp/common/ecp_g_visioncoordinates.h"
#include "ecp_mp/ecp_mp_s_cvfradia.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

#define debugmsg(msg)						task::task::sr_ecp_msg->message(msg);

////////////////////////////////////////////////////////////////////////////////////////////
// ecp_visioncoordinates_generator
////////////////////////////////////////////////////////////////////////////////////////////
visioncoordinates::visioncoordinates(common::task::task& _ecp_task)
: generator(_ecp_task), SETTINGS_SECTION_NAME("[ecp_visioncoordinates_generator]")
{
	debugmsg("VCG: Creating virtual sensor to communicate with FraDIA");
	sensor_m[lib::SENSOR_CVFRADIA] = new ecp_mp::sensor::cvfradia(lib::SENSOR_CVFRADIA, SETTINGS_SECTION_NAME, ecp_t, sizeof(lib::sensor_image_t::sensor_union_t::visioncoordinates_t));
	sensor_m[lib::SENSOR_CVFRADIA]->configure_sensor();

	debugmsg("VCG: Sensor configured");

}

bool visioncoordinates::first_step()
{
	// przygotowywujemy sie do pobrania danych z robota
	debugmsg("VCG: first_step()");

	if (!the_robot)
		debugmsg("VCG: the robot not exists");

	ecp_mp::robot_transmission_data& data = the_robot->EDP_data;

	debugmsg("VCG: robot_transmission_data ready");

	data.instruction_type = lib::GET;

	debugmsg("VCG: setting instruction type GET done");
	data.get_type = ARM_DV;
	data.get_arm_type = lib::XYZ_ANGLE_AXIS;	// XYZ_EULER_ZYZ;
	data.motion_type = lib::ABSOLUTE;
	data.next_interpolation_type = lib::MIM;

	debugmsg("VCG: Ready to get data");

	return true;
}

void describe_matrix(lib::Homog_matrix& matrix, const char* name)
{
	std::ostringstream oss1;
	oss1 << "##### Matrix " << name << ":" << std::endl;
	oss1 << matrix << std::flush;
	debugmsg(oss1.str().c_str());

	double t[8];
	std::ostringstream oss;
	oss << std::endl << "xyz_angle_axis: ";
	memset(t, 0, sizeof(t));
	matrix.get_xyz_angle_axis(t);
	for (int i = 0; i < sizeof(t)/sizeof(t[0]); ++i)
	{
		if (i == 3 || i == 6) 
			oss << "| ";
		oss << t[i] << " ";
	}
	oss << std::endl;

	oss << "xyz_euler_zyz: ";
	memset(t, 0, sizeof(t));
	matrix.get_xyz_euler_zyz(t);
	for (int i = 0; i < sizeof(t)/sizeof(t[0]); ++i)
	{
		if (i == 3 || i == 6) 
			oss << "| ";
		oss << t[i] << " ";
	}
	oss << std::endl;

	debugmsg(oss.str().c_str());
}

bool visioncoordinates::next_step()
{
	debugmsg("VCG: Processing data");

	lib::SENSOR_IMAGE& sensor = sensor_m[lib::SENSOR_CVFRADIA]->image;
	const double xoz = sensor.sensor_union.visioncoordinates.xOz;
	const double z = sensor.sensor_union.visioncoordinates.z;

	ecp_mp::robot_transmission_data& data = the_robot->EDP_data;

	std::ostringstream oss;

	oss << "coordinates: ";
	for (int i = 0; i < sizeof(data.current_XYZ_ZYZ_arm_coordinates)/sizeof(data.current_XYZ_ZYZ_arm_coordinates[0]); ++i)
		oss << data.current_XYZ_ZYZ_arm_coordinates[i] << " ";
	oss << std::endl;

	//double* arm = &data.current_XYZ_ZYZ_arm_coordinates[0];
	double* arm = &data.current_XYZ_AA_arm_coordinates[0];

	// current_XYZ_AA_arm_coordinates zawiera 8 elementow, wykorzystujemy 6 pierwszych xyz_zyz
	lib::Homog_matrix current_position(lib::Homog_matrix::MTR_XYZ_ANGLE_AXIS, arm[0], arm[1], arm[2], arm[3], arm[4], arm[5]); // aktualna pozycja ramienia robota
	lib::Homog_matrix move(lib::Homog_matrix::MTR_XYZ_EULER_ZYZ, 0.0, 0.0, 0.0, z, xoz, -z); 


	describe_matrix(current_position, "current_position");
	describe_matrix(move, "move");


	oss << "VCG current_position: " << std::endl << current_position << std::flush;
	debugmsg(oss.str().c_str());


	std::ostringstream oss2;
	oss2 << "move:  xoz: " << xoz << " z: " << z << std::endl << move << std::endl;
	debugmsg(oss2.str().c_str());


	lib::Homog_matrix target = current_position * move;

	std::ostringstream oss3;
	oss3 << "target: " << std::endl << target << std::endl;
	describe_matrix(target, "target = current_position * move");
	
	debugmsg(oss3.str().c_str());

	//memcpy(itsOutputCoordinates, arm, sizeof(data.current_XYZ_AA_arm_coordinates));
	//target.get_xyz_angle_axis(itsOutputCoordinates);
	memcpy(itsOutputCoordinates, arm, sizeof(data.current_XYZ_ZYZ_arm_coordinates));
	target.get_xyz_euler_zyz(itsOutputCoordinates);
	itsOutputCoordinates[6] = data.current_gripper_coordinate;

	std::ostringstream oss4;
	oss4 << "output coordinates: ";
	for (int i = 0; i < sizeof(itsOutputCoordinates) / sizeof(itsOutputCoordinates[0]); ++i)
		oss4 << itsOutputCoordinates[i] << " ";

	debugmsg(oss4.str().c_str());

	debugmsg("VCG: Data processed");

	return false;		// koniec pracy generatora, teraz zadanie powinno wywolac generator smooth
}

#undef debugmsg

} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp

