// ecp_t_visioncoordinates.cc - definiuje zadanie podazania chwytaka (kamery) robota w strone obiektu
//    zlokalizowanego na ekranie
// (c)2009 Maciej Jerzy Nowak
// Created: 20.02.2009  Last Update: 20.02.2009
///////////////////////////////////////////////////////////////////////////////

//#include <stdio.h>
#include <string.h>
//#include <unistd.h>
#include <sstream>

#include "common/typedefs.h"
#include "common/impconst.h"
#include "common/com_buf.h"

#include "lib/srlib.h"

#include "ecp/irp6_on_track/ecp_local.h"
#include "ecp/irp6_postument/ecp_local.h"

#include "ecp_mp/ecp_mp_s_cvfradia.h"
#include "ecp/common/ecp_t_visioncoordinates.h"

namespace mrrocpp {
namespace ecp {
namespace common {

#define debugmsg(msg)		sr_ecp_msg->message(msg);
/////////////////////////////////////////////////////////////////////////////////////////////
// ecp_task_visioncoordinates
/////////////////////////////////////////////////////////////////////////////////////////////

ecp_task_visioncoordinates::ecp_task_visioncoordinates(configurator& _config)
	: ecp_task(_config), SETTINGS_SECTION_NAME("[ecp_visioncoordinates_task]")
{
}

void ecp_task_visioncoordinates::task_initialization()
{

	// tworzymy robota
    if (strcmp(config.section_name, "[ecp_irp6_on_track]") == 0)
    {
        ecp_m_robot = new ecp_irp6_on_track_robot (*this);
        debugmsg("IRp6ot loaded");
    }
    else if (strcmp(config.section_name, "[ecp_irp6_postument]") == 0)
    {
        ecp_m_robot = new irp6p::ecp_irp6_postument_robot(*this);
        debugmsg("IRp6p loaded");
    }

	// i powiazane z nim generatory - czucia wizji i ruchu :)
	itsVisionGen = new ecp_visioncoordinates_generator(*this);
	itsSmoothGen = new ecp_smooth_generator(*this, true, false); // synchronized, debug
}

void ecp_task_visioncoordinates::main_task_algorithm()
{
	debugmsg("main_task_alogrithm");

	setStartPosition();

	double bf[8];			// bufor na wspolrzedne prezkazywane do generatora smooth
	int loop = 5;
	while (loop--)
	{
		debugmsg("itsVisionGen->Move()");
		itsVisionGen->Move();
		debugmsg("getNewCoordinates()");
		itsVisionGen->getNewCoordinates(bf);
		debugmsg("load_coordinates()");
		itsSmoothGen->load_coordinates(XYZ_EULER_ZYZ, bf[0], bf[1], bf[2], bf[3], bf[4], bf[5], bf[6], bf[7]);
		debugmsg("itsSmoothGen->Move()");
		itsSmoothGen->Move();
		debugmsg("itsShoothGen->reset()");
		itsSmoothGen->reset();
	}
}

void ecp_task_visioncoordinates::setStartPosition()
{
	debugmsg("setStartPosition()");
	double bf[MAX_SERVOS_NR]; 
	memset(bf, 0, sizeof(bf));
	char* position = config.return_string_value("start_joint_position", SETTINGS_SECTION_NAME);

	std::istringstream iss(position);
	iss >> bf[0] >> bf[1] >> bf[2] >> bf[3] >> bf[4] >> bf[5] >> bf[6]; 
	delete[] position;

	itsSmoothGen->load_coordinates(JOINT, bf[0], bf[1], bf[2], bf[3], bf[4], bf[5], bf[6], bf[7]);
	itsSmoothGen->Move();
	itsSmoothGen->reset();
}

/////////////////////////////////////////////////////////////////////////////////////////////
// fabryk abstrakcyjna dla zada�
/////////////////////////////////////////////////////////////////////////////////////////////
ecp_task* return_created_ecp_task (configurator &_config)
{
	return new ecp_task_visioncoordinates(_config);
}

#undef debugmsg



} // namespace common
} // namespace ecp
} // namespace mrrocpp

