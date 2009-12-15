/// \file ecp_t_visioncoordinates.cc
/// \brief definiuje zadanie podazania chwytaka (kamery) robota w strone obiektu zlokalizowanego na ekranie
/// \author Maciej Jerzy Nowak
/// \date 2009.07.22
///////////////////////////////////////////////////////////////////////////////

#include <string.h>
#include <unistd.h>
#include <sstream>

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "lib/srlib.h"

#include "ecp/irp6_on_track/ecp_r_irp6ot.h"
#include "ecp/irp6_postument/ecp_r_irp6p.h"

#include "ecp_mp/sensor/ecp_mp_s_cvfradia.h"
#include "ecp/common/task/ecp_t_visioncoordinates.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace task {

#define debugmsg(msg)		sr_ecp_msg->message(msg)
/////////////////////////////////////////////////////////////////////////////////////////////
// ecp_task_visioncoordinates
/////////////////////////////////////////////////////////////////////////////////////////////

visioncoordinates::visioncoordinates(lib::configurator& _config)
	: task(_config), SETTINGS_SECTION_NAME("[ecp_visioncoordinates_task]")
{
	// tworzymy robota
    if (config.section_name == ECP_IRP6_ON_TRACK_SECTION)
    {
        ecp_m_robot = new irp6ot::robot (*this);
        debugmsg("IRp6ot loaded");
    }
    else if (config.section_name == ECP_IRP6_POSTUMENT_SECTION)
    {
        ecp_m_robot = new irp6p::robot(*this);
        debugmsg("IRp6p loaded");
    }

	// i powiazane z nim generatory - czucia wizji i ruchu :)

	itsVisionGen = new generator::visioncoordinates(*this);
	itsSmoothGen = new generator::smooth(*this, true, false); // synchronized, debug
}

void visioncoordinates::main_task_algorithm()
{
	debugmsg("main_task_alogrithm");

	double bf[8];			// bufor na wspolrzedne prezkazywane do generatora smooth

	// --- sterowania dla generatora smooth, by robot wolniej przyspieszal ---
	double vp[MAX_SERVOS_NR]={0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	double vk[MAX_SERVOS_NR]={0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	double v[MAX_SERVOS_NR]={0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 5.0};
	double a[MAX_SERVOS_NR]={0.03, 0.03, 0.03, 0.03, 0.03, 0.03, 0.03, 0.5};

	while (true)
	{
		// pozwalamy uzytkownikowi wybrac, jaki obiekt mamy rozpoznac/czy zakonczyc zadanie
		if (!selectObject())
			return;		// nie wybralismy obiektu => konczymy zadanie

		setStartPosition();
		sleep(1);

		itsVisionGen->Move();		// wykonujemy ruch, polegajacy na rozpoznaniu okolicy :)

		bool found = false;
		while (itsVisionGen->getCoordinates(bf))
		{
			debugmsg("sa nowe wspolrzedne...");
			// --- po pobraniu wsp�rz�dnych wykonujemy ruch ---
			//itsSmoothGen->load_coordinates(lib::XYZ_EULER_ZYZ, bf[0], bf[1], bf[2], bf[3], bf[4], bf[5], bf[6], bf[7]);
			itsSmoothGen->load_coordinates(lib::XYZ_EULER_ZYZ, vp, vp, v, a, bf);
			itsSmoothGen->Move();
			itsSmoothGen->reset();

			// --- czekamy a� chwytak si� ustabilizuje ---
			sleep(1);

			// --- sprawdzamy, czy widzimy dany obiekt ---
			debugmsg("test");
			if (itsVisionGen->test() || // test wykonujemy podwojnie
				itsVisionGen->test())   // by wyeliminowac ew. zakolcenia
			{
				found = true;
				break;		// obiekt zostal znaleziony, nie szukamy nastepnych obiektow
			}

			debugmsg("niestety to nie jest ten obiekt");
		}

		if (found)
			debugmsg("Znaleziono poszukiwany obiekt!");
		else
			debugmsg("Poszukiwanego obiektu NIE znaleziono :-(");
	}

/*	double bf[8];			// bufor na wspolrzedne prezkazywane do generatora smooth

	int loop = 5;
	while (loop--)
	{
		debugmsg("itsVisionGen->Move()");
		itsVisionGen->Move();
		debugmsg("getNewCoordinates()");
		itsVisionGen->getNewCoordinates(bf);
		debugmsg("load_coordinates()");
		itsSmoothGen->load_coordinates(lib::XYZ_EULER_ZYZ, bf[0], bf[1], bf[2], bf[3], bf[4], bf[5], bf[6], bf[7]);
		debugmsg("itsSmoothGen->Move()");
		itsSmoothGen->Move();
		debugmsg("itsShoothGen->reset()");
		itsSmoothGen->reset();

	}*/
}

void visioncoordinates::setStartPosition()

{
	debugmsg("setStartPosition()");

	// --- sterowania dla generatora smooth, by robot wolniej przyspieszal ---
	double vp[MAX_SERVOS_NR]={0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	double vk[MAX_SERVOS_NR]={0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	double v[MAX_SERVOS_NR]={0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 5.0};
	double a[MAX_SERVOS_NR]={0.03, 0.03, 0.03, 0.03, 0.03, 0.03, 0.03, 0.5};

	double bf[MAX_SERVOS_NR];
	memset(bf, 0, sizeof(bf));

	std::string position = config.return_string_value("start_joint_position", SETTINGS_SECTION_NAME);


	std::istringstream iss(position.c_str());
	iss >> bf[0] >> bf[1] >> bf[2] >> bf[3] >> bf[4] >> bf[5] >> bf[6];


	itsSmoothGen->load_coordinates(lib::JOINT, vp, vk, v, a, bf);

	itsSmoothGen->Move();

	itsSmoothGen->reset();

}

bool visioncoordinates::selectObject()
{
	// wyb�r opcji "czego poszukujemy" - po stronie mrroc++, na podstawie obiektow z FraDIA
	std::ostringstream msg;
	const std::vector<std::string>& known = itsVisionGen->knownObjects();
	for (int opt = 0; opt < known.size(); ++opt)
		msg << " " << (opt + 1) << ". " << known[opt];


	// GUI MRROC++ pozwala wybrac uzytkownikowi tylko max. z 4 obiektow - do celow demonstracyjnych to wystarczy
	int option = choose_option(msg.str().c_str(), std::min<int>(known.size(), 4));
	switch (option)
	{
	case lib::OPTION_ONE: itsVisionGen->searchObject(0); break;
	case lib::OPTION_TWO: itsVisionGen->searchObject(1); break;
	case lib::OPTION_THREE: itsVisionGen->searchObject(2); break;
	case lib::OPTION_FOUR: itsVisionGen->searchObject(3); break;
	default:
		setStartPosition();
		return false;
	}
	return true;
}

/////////////////////////////////////////////////////////////////////////////////////////////
// fabryk abstrakcyjna dla zadan
/////////////////////////////////////////////////////////////////////////////////////////////
task* return_created_ecp_task (lib::configurator &_config)
{
	return new visioncoordinates(_config);
}


#undef debugmsg



} // namespace task
} // namespace common
} // namespace ecp
} // namespace mrrocpp

