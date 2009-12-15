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

#include "ecp/common/generator/ecp_g_visioncoordinates.h"
#include "ecp_mp/sensor/ecp_mp_s_cvfradia.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

#define debugmsg(msg)						sr_ecp_msg.message(msg)

////////////////////////////////////////////////////////////////////////////////////////////
// ecp_visioncoordinates_generator
////////////////////////////////////////////////////////////////////////////////////////////
visioncoordinates::visioncoordinates(common::task::task& _ecp_task)
: generator(_ecp_task), SETTINGS_SECTION_NAME("[ecp_visioncoordinates_generator]")
{
	debugmsg("ecp_g_visioncoordinates: Creating virtual sensor to communicate with FraDIA");
	sensor_m[lib::SENSOR_CVFRADIA] = new ecp_mp::sensor::cvfradia(lib::SENSOR_CVFRADIA, SETTINGS_SECTION_NAME, ecp_t, sizeof(lib::sensor_image_t::sensor_union_t::visioncoordinates_union_t));
	sensor_m[lib::SENSOR_CVFRADIA]->configure_sensor();

	sensor_in = &sensor_m[lib::SENSOR_CVFRADIA]->image;
	sensor_out = &sensor_m[lib::SENSOR_CVFRADIA]->to_vsp;

	getKnownObjects();

	debugmsg("ecp_g_visioncoordinates: Sensor configured");
}

bool visioncoordinates::first_step()
{
	// przygotowywujemy sie do pobrania polozenia robota
	if (!the_robot)
		debugmsg("ecp_g_visioncoordinates: the robot not exists");

	// --- przygotowywujemy sie do pobrania polozenia robota w ramach Move() -> execute_motion() ---

	//ecp_mp::robot_transmission_data& data = the_robot->ecp_command.instruction;

	debugmsg("ecp_g_visioncoordinates: robot_transmission_data ready");

	the_robot->ecp_command.instruction.instruction_type = lib::GET;
	the_robot->ecp_command.instruction.get_type = ARM_DV;
	the_robot->ecp_command.instruction.get_arm_type = lib::XYZ_ANGLE_AXIS;
	the_robot->ecp_command.instruction.motion_type = lib::ABSOLUTE;
	the_robot->ecp_command.instruction.interpolation_type = lib::MIM;

	// FraDIA ma znale�� wszystkie obiekty, nawet troch� podobne do poszukiwanego
	sensor_out->esa.mode = lib::EM_SEARCH;
	strcpy(sensor_out->esa.object, itsSearchObject.c_str());

	debugmsg("ecp_g_visioncoordinates: Ready to get data");
	return true;
}

#if 0
// TODO: this should be private method, which can access generator::sr_ecp_msg object

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
#endif

lib::Homog_matrix visioncoordinates::calculateMove(double rot_z, double rot_dev, double distance)
{
	// nie wiem jak narazie policzyc dystans, o ktory sie powinnismy przesunac. dystanss powinien byc niewielki,
	// najlepiej ograniczony z gory, by nie narobic szkody robotem np. wbijajac sie w stol gdy zobaczymy plamke,
	// zinterpretowana jako widziany z daleka sztuciec. Jesli za bardzo sie przysuniemy do sztucca, to tez nie zobaczymy
	// go w calosci. Otrzymany dystans powinien byc wiec dystansem o ktory mamy sie przyblizyc, np. poprzez
	// skalowanie ROI tak, by miescil sie na 2/3 ekranu (jego najszersza przekatna wzgledem wysokosci obrazu otrzymanego z kamery)
	// etc - tylko ze to sie powinno dziac ze stronie fradii, tutaj powinno byc tylko ograniczenie na max. wysiegnik

	const double MAX_DISTANCE = 0.20; // [m]
	if (distance > MAX_DISTANCE)
		distance = MAX_DISTANCE;

	lib::Homog_matrix move_z(lib::Homog_matrix::MTR_XYZ_ANGLE_AXIS, 0.0, 0.0, 0.0,				0.0, 0.0, rot_z);
	lib::Homog_matrix move_y(lib::Homog_matrix::MTR_XYZ_ANGLE_AXIS, 0.0, 0.0, 0.0,				0.0, rot_dev, 0.0);
	lib::Homog_matrix move_dist(lib::Homog_matrix::MTR_XYZ_ANGLE_AXIS, 0.0, 0.0, distance,		0.0, 0.0, 0.0);

	lib::Homog_matrix move_back_z(lib::Homog_matrix::MTR_XYZ_ANGLE_AXIS, 0.0, 0.0, 0.0,		0.0, 0.0, -rot_z);			// narazie przywracamy istniejacy kierunek

	return move_z * move_y * move_dist * move_back_z;
}

bool visioncoordinates::next_step()
{
	typedef lib::sensor_image_t::sensor_union_t::visioncoordinates_union_t::Search Search;

	debugmsg("ecp_g_visioncoordinates: Processing data");

	//ecp_mp::robot_transmission_data& data = the_robot->EDP_data;

	// current_XYZ_AA_arm_coordinates zawiera 6 elementow, chwytak (gripper) jest osobno
	lib::Homog_matrix current_position(lib::Homog_matrix::MTR_XYZ_ANGLE_AXIS, the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates); // aktualna pozycja ramienia robota

	itsCoordinates.clear();
	for (int it = 0; it < 8 /* sizeof(sensor_in->sensor_union.visioncoordinates_union.search) / sizeof(sensor_in->sensor_union.visioncoordinates_union.search[0])*/ ; ++it)
	{
		Search* search = &(sensor_in->sensor_union.visioncoordinates_union.search[it]);
		if (search->dist == 0.0)
			break;
		lib::Homog_matrix move = calculateMove(search->rot_z, search->rot_dev, search->dist);
		lib::Homog_matrix target = current_position * move;

		EulerCoordinates ec;
		target.get_xyz_euler_zyz(ec.bf);
		ec.bf[6] = the_robot->reply_package.arm.pf_def.gripper_coordinate;
		itsCoordinates.push_back(ec);
	}

	return false;			// koniec pracy generatora, teraz zadanie powinno wywolac generator smooth
}

bool visioncoordinates::getCoordinates(double output[8])
{
	if (itsCoordinates.size() == 0)			// nie ma wiecej elementow do sprawdzenia
		return false;

	itsCoordinates.front().to(output);	// jest jeszcze cos do sprawdzenia - kopiujemy
	itsCoordinates.pop_front();			// i bysmy ponownie nie uzywali tych wspolrzednych

	return true;			// przekazane wspolrzedne moga zostac sprawdzone
}

bool visioncoordinates::test()
{
	typedef lib::sensor_image_t::sensor_union_t::visioncoordinates_union_t::Test Test;

	sensor_out->esa.mode = lib::EM_TEST;
	strcpy(sensor_out->esa.object, itsSearchObject.c_str());
	sensor_m[lib::SENSOR_CVFRADIA]->get_reading();
	Test* ret = &(sensor_in->sensor_union.visioncoordinates_union.test);
	return ret->found;
}

void visioncoordinates::getKnownObjects()
{
	typedef lib::sensor_image_t::sensor_union_t::visioncoordinates_union_t::List List;

	sensor_out->esa.mode = lib::EM_LIST;
	sensor_out->esa.offset = 0;
	sensor_m[lib::SENSOR_CVFRADIA]->get_reading();
	List* ret = &(sensor_in->sensor_union.visioncoordinates_union.list);
	itsKnownObjects.resize(ret->count);
	int it = 0;

	while (it < ret->count)
	{
		itsKnownObjects[it] = std::string(ret->object[it % 8]);
		std::cout << "we known object " << itsKnownObjects[it] << std::endl;

		++it;

		if (it % 8 == 0)				// --- byc moze musimy pobrac kolejna porcje danych
		{
			sensor_out->esa.mode = lib::EM_LIST;
			sensor_out->esa.offset = it;
			sensor_m[lib::SENSOR_CVFRADIA]->get_reading();
			ret = &(sensor_in->sensor_union.visioncoordinates_union.list);
		}
	}
}

/*
	const double xoz = sensor_in->sensor_union.visioncoordinates.xOz;
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

	//lib::Homog_matrix move(lib::Homog_matrix::MTR_XYZ_EULER_ZYZ, 0.0, 0.0, 0.0, z, xoz, -z);

	//
	// z wyznacza nam kolejny numer argumentu, do ktorego podamy sterowanie w xoz
	// to w celu przetestowania kolejnych osi chwytaka -- gdy juz ustalimy osie chwytaka, i przetestujemy obroty wokol nich
	//
	// natepnym testem po ustaleniu osi obrotow, bedzie dodanie przesuniecia wzgledem osi chwytaka z (ruch do przodu po skierowaniu sie
	// w strone chwytaka)
	//
	// nastepnym krokiem moze byc sprawdzenie zachowania sie robota przy troche innym polozeniu poczatkowym (moze tak, by robot nie mial za bardzo mozliwosci
	// zmiany orientacji chwytaka na inna - w sensie by nie wystepowaly niejednoznacznosci. oprocz tego kwestia osobna jest gdy odchyla sie na bok - robot na bok odchylic sie nie
	// moze, ale jesli dostanie rotacje ze nie musi odchylac sie na bok, to przeciez nie bedzie probowal... Czyli podsumowujac - niech nasz ,,bocian'' patrzy bardziej w dol
	// i dla takiego polozenia poczatkowego (gdy nie bedzie niejednoznacznosci) sprawdzac czy rotacje rusza.
	int mode = int(z + 0.1);

	debugmsg("move in MTR_XYZ_ANGLE_AXIS");
	lib::Homog_matrix move(lib::Homog_matrix::MTR_XYZ_ANGLE_AXIS,
			(mode == 0) ? xoz : 0.0,
			(mode == 1) ? xoz : 0.0,
			(mode == 2) ? xoz : 0.0,
			(mode == 3) ? xoz : 0.0,
			(mode == 4) ? xoz : 0.0,
			(mode == 5) ? xoz : 0.0);
	//		*/
/*
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
}*/

#undef debugmsg

} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp

