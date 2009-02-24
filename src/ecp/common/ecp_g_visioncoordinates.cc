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

#include "common/typedefs.h"
#include "common/impconst.h"
#include "common/com_buf.h"

#include "lib/srlib.h"
#include "lib/mathtr.h"

#include "ecp/common/ecp_g_visioncoordinates.h"
#include "ecp_mp/ecp_mp_s_cvfradia.h"

#define debugmsg(msg)						ecp_task::sr_ecp_msg->message(msg);

////////////////////////////////////////////////////////////////////////////////////////////
// ecp_visioncoordinates_generator
////////////////////////////////////////////////////////////////////////////////////////////
ecp_visioncoordinates_generator::ecp_visioncoordinates_generator(ecp_task& _ecp_task)
: ecp_generator(_ecp_task), SETTINGS_SECTION_NAME("[ecp_visioncoordinates_generator]")
{
	debugmsg("VCG: Creating virtual sensor to communicate with FraDIA");
	sensor_m[SENSOR_CVFRADIA] = new ecp_mp_cvfradia_sensor(SENSOR_CVFRADIA, SETTINGS_SECTION_NAME, ecp_t, sizeof(sensor_image_t::sensor_union_t::visioncoordinates_t));
	sensor_m[SENSOR_CVFRADIA]->configure_sensor();

	debugmsg("VCG: Sensor configured");

}

bool ecp_visioncoordinates_generator::first_step()
{
	// przygotowywujemy sie do pobrania danych z robota
	debugmsg("VCG: first_step()");

	if (!the_robot)
		debugmsg("VCG: the robot not exists");

	robot_transmission_data& data = the_robot->EDP_data;

	debugmsg("VCG: robot_transmission_data ready");

	data.instruction_type = GET;

	debugmsg("VCG: setting instruction type GET done");
	data.get_type = ARM_DV;
	data.get_arm_type = XYZ_EULER_ZYZ;
	data.motion_type = ABSOLUTE;
	data.next_interpolation_type = MIM;

	debugmsg("VCG: Ready to get data");

	return true;
}

bool ecp_visioncoordinates_generator::next_step()
{
	debugmsg("VCG: Processing data");

	SENSOR_IMAGE& sensor = sensor_m[SENSOR_CVFRADIA]->image;
	const double xoz = sensor.sensor_union.visioncoordinates.xOz;
	const double z = sensor.sensor_union.visioncoordinates.z;

	robot_transmission_data& data = the_robot->EDP_data;

	std::ostringstream oss;

	oss << "coordinates: ";
	for (int i = 0; i < sizeof(data.current_XYZ_ZYZ_arm_coordinates)/sizeof(data.current_XYZ_ZYZ_arm_coordinates[0]); ++i)
		oss << data.current_XYZ_ZYZ_arm_coordinates[i] << " ";
	oss << std::endl;

	double* arm = &data.current_XYZ_ZYZ_arm_coordinates[0];

	// current_XYZ_AA_arm_coordinates zawiera 8 elementow, wykorzystujemy 6 pierwszych xyz_zyz
	Homog_matrix current_position(Homog_matrix::MTR_XYZ_EULER_ZYZ, arm[0], arm[1], arm[2], arm[3], arm[4], arm[5]); // aktualna pozycja ramienia robota
	Homog_matrix move(Homog_matrix::MTR_XYZ_EULER_ZYZ, 0.0, 0.0, 0.0, 0.0, xoz, z); 


	oss << "VCG current_position: " << std::endl << current_position << std::flush;
	debugmsg(oss.str().c_str());


	std::ostringstream oss2;
	oss2 << "move:  xoz: " << xoz << " z: " << z << std::endl << move << std::endl;
	debugmsg(oss2.str().c_str());


	Homog_matrix target = current_position * move;

	std::ostringstream oss3;
	oss3 << "target: " << std::endl << target << std::endl;
	
	debugmsg(oss3.str().c_str());

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

