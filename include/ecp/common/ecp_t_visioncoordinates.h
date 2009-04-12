// ecp_t_visioncoordinates.h - definicja zadania podazania w strone zidentyfikowanego obiektu na ekranie
// (c)2009 Maciej Jerzy Nowak
// Created: 20.02.2009  Last Update: 20.02.2009
///////////////////////////////////////////////////////////////////////////////


#ifndef ECP_T_VISIONCOORDINATES_H_INCLUDED
#define ECP_T_VISIOnCOORDINATES_H_INCLUDED


#include "ecp/common/ecp_task.h"
#include "ecp/common/ecp_g_visioncoordinates.h"
#include "ecp/common/ecp_g_smooth.h"

namespace mrrocpp {
namespace ecp {
namespace common {

// class ecp_task_visioncoordinates : public common::ecp_task
// - zadanie kierowania ramieniem robota w strone zidentyfikowanego obiektu na ekranie
class ecp_task_visioncoordinates : public common::ecp_task  
{
public:
	ecp_task_visioncoordinates(configurator& _config);

	// void task_initialization()
	// - Initialize task - robot, sensors and generators.
	void task_initialization();

	// void main_task_algorithm()
	// -  Main algorithm loop.
	void main_task_algorithm();
	

private:
	// void setStartPosition()
	// ustawia poczatkowa pozycje (z kamera powyzje ramienia robota)
	// na podstawie pozycji zadanej w pliku konfiguracyjnym
	void setStartPosition();

	ecp_visioncoordinates_generator* itsVisionGen;
	ecp_smooth_generator* itsSmoothGen;

	// nazwa sekcji zawierajacej ustawienia dla zadania
	const char* SETTINGS_SECTION_NAME;
};

} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif	// ECP_T_VISIONCOORDINATES_H_INCLUDED
