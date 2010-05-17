// -------------------------------------------------------------------------
//                            ecp_mp_sensor.cc
//            Effector Control Process (lib::ECP) i MP - methods
//
// Wlasciwy konstruktor czujnika wirtualnego.
// -------------------------------------------------------------------------

#include "ecp_mp/sensor/ecp_mp_sensor.h"
#include "ecp_mp/task/ecp_mp_task.h"

#if defined(USE_MESSIP_SRR)
#include "messip_dataport.h"
#endif

#include <iostream>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <string.h>

namespace mrrocpp {
namespace ecp_mp {
namespace sensor {
#if 0
template <typename SENSOR_IMAGE, typename CONFIGURE_DATA = void *>
sensor::sensor(lib::SENSOR_t _sensor_name, const std::string & _section_name, task::task& _ecp_mp_object)
	: sr_ecp_msg(*_ecp_mp_object.sr_ecp_msg), sensor_name(_sensor_name)
{
	// cout<<"ecp_mp_sensor - konstruktor: "<<_section_name<<endl;

	// Ustawienie domyslnego okresu pracy czujnika.
	base_period = current_period = 1;

	node_name = _ecp_mp_object.config.value<std::string>("node_name", _section_name);

#if !defined(USE_MESSIP_SRR)
	VSP_NAME = _ecp_mp_object.config.return_attach_point_name(lib::configurator::CONFIG_RESOURCEMAN_GLOBAL, "resourceman_attach_point", _section_name);

 	// cout<<"VSP_NAME = "<<VSP_NAME<<endl;

	// Sprawdzeie czy nie jest juz zarejestrowany zarzadca zasobow o tej nazwie.
	if( access(VSP_NAME.c_str(), R_OK)==0 )
	{
		// by Y - usuniete bo mozna podlaczyc sie do istniejacego czujnika
		// throw sensor_error(lib::SYSTEM_ERROR, DEVICE_ALREADY_EXISTS);
		pid = 0; // tymczasowo
	} else {
	// Stworzenie nowego procesu.
	if ((pid = _ecp_mp_object.config.process_spawn(_section_name)) == -1)
		throw sensor_error(lib::SYSTEM_ERROR, CANNOT_SPAWN_VSP);
		// Proba otworzenie urzadzenia.
	}

	short tmp = 0;
 	// Kilka sekund  (~2) na otworzenie urzadzenia.
	while( (sd = open(VSP_NAME.c_str(), O_RDWR)) == -1)
	{
// 		cout<<tmp<<endl;
		if((tmp++)<CONNECT_RETRY)
			usleep(1000*CONNECT_DELAY);
		else
			throw sensor_error(lib::SYSTEM_ERROR, CANNOT_LOCATE_DEVICE);
	}
#else /* USE_MESSIP_SRR */

	VSP_NAME = _ecp_mp_object.config.return_attach_point_name(lib::configurator::CONFIG_SERVER, "resourceman_attach_point", _section_name);

 	// cout<<"VSP_NAME = "<<VSP_NAME<<endl;

	// Sprawdzeie czy nie jest juz zarejestrowany zarzadca zasobow o tej nazwie.
	if( (sd = messip::port_connect(VSP_NAME)))
	{
		// by Y - usuniete bo mozna podlaczyc sie do istniejacego czujnika
		// throw sensor_error(lib::SYSTEM_ERROR, DEVICE_ALREADY_EXISTS);
		pid = 0; // tymczasowo
		return;
	}

	// Stworzenie nowego procesu.
	if ((pid = _ecp_mp_object.config.process_spawn(_section_name)) == -1)
		throw sensor_error(lib::SYSTEM_ERROR, CANNOT_SPAWN_VSP);

	short tmp = 0;
 	// Kilka sekund  (~2) na otworzenie urzadzenia.
	while( (sd = messip::port_connect(VSP_NAME)) == NULL)
	{
// 		cout<<tmp<<endl;
		if((tmp++)<CONNECT_RETRY)
			usleep(1000*CONNECT_DELAY);
		else {
			std::cerr << "ecp_mp_sensor: messip::port_connect(" << VSP_NAME << ") failed" << std::endl;
			throw sensor_error(lib::SYSTEM_ERROR, CANNOT_LOCATE_DEVICE);
		}
	}// end: while
#endif /* !USE_MESSIP_SRR */
}

#endif

} // namespace sensor
} // namespace ecp_mp
} // namespace mrrocpp

