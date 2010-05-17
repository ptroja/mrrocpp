#if !defined(_ECP_MP_SENSOR_H)
#define _ECP_MP_SENSOR_H

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include "lib/sensor.h"
#include "lib/srlib.h"
#include "lib/configurator.h"

#if defined(USE_MESSIP_SRR)
#include <messip.h>
#endif

#include <map>

namespace mrrocpp {
namespace ecp_mp {
namespace sensor {

class sensor_interface : public lib::sensor_interface
{
public:
	// ponizsze zmienne pozwalaja na odczyty z roznym okresem z czujnikow (mierzonym w krokach generatora)
	// w szczegolnosci mozliwe jest unikniecie odczytu po first stepie (nalezy base_period ustawic na 0)
	short base_period; // by Y okresla co ile krokow generatora ma nastapic odczyt z czujnika
	short current_period; // by Y ilosc krokow pozostajaca do odczytu z czujnika

	//! nazwa wezla na ktorym jest powolane vsp
	std::string node_name;
};

// Klasa bazowa czujnikow po stronie procesu ECP.
template <typename SENSOR_IMAGE, typename CONFIGURE_DATA = lib::empty_t>
class sensor : public sensor_interface {
private:
	//! VSP process id
	pid_t pid; // pid vsp

	//! Sensor manger descriptor
#if !defined(USE_MESSIP_SRR)
	int sd;
#else
	//! Sensor descriptor
	messip_channel_t *sd;
#endif /* USE_MESSIP_SRR */
	// Nazwa czujnika.
	std::string VSP_NAME;

	// Wskaznik na obiekt do komunikacji z SR
	lib::sr_ecp &sr_ecp_msg;

	//! Buffer for data to VSP
	struct VSP_ECP_MSG
	{
		lib::VSP_REPORT_t vsp_report;
		SENSOR_IMAGE comm_image;
	} from_vsp;

public:
	//! Image of the sensor
	SENSOR_IMAGE image;

	//! Buffer for data from VSP
	struct ECP_VSP_MSG {
		lib::VSP_COMMAND_t i_code;
		CONFIGURE_DATA command;
	} to_vsp;

	const lib::SENSOR_t sensor_name; // nazwa czujnika z define w impconst.h

	// Wlasciwy konstruktor czujnika wirtualnego.
	sensor(lib::SENSOR_t _sensor_name, const std::string & _section_name, lib::sr_ecp & _sr_ecp_msg, lib::configurator & config)
		: sr_ecp_msg(_sr_ecp_msg), sensor_name(_sensor_name)
	{
		// cout<<"ecp_mp_sensor - konstruktor: "<<_section_name<<endl;

		// Ustawienie domyslnego okresu pracy czujnika.
		base_period = current_period = 1;

		node_name = config.value<std::string>("node_name", _section_name);

	#if !defined(USE_MESSIP_SRR)
		VSP_NAME = config.return_attach_point_name(lib::configurator::CONFIG_RESOURCEMAN_GLOBAL, "resourceman_attach_point", _section_name);

	 	// cout<<"VSP_NAME = "<<VSP_NAME<<endl;

		// Sprawdzeie czy nie jest juz zarejestrowany zarzadca zasobow o tej nazwie.
		if( access(VSP_NAME.c_str(), R_OK)==0 )
		{
			// by Y - usuniete bo mozna podlaczyc sie do istniejacego czujnika
			// throw sensor_error(lib::SYSTEM_ERROR, DEVICE_ALREADY_EXISTS);
			pid = 0; // tymczasowo
		} else {
		// Stworzenie nowego procesu.
		if ((pid = config.process_spawn(_section_name)) == -1)
			throw lib::sensor::sensor_error(lib::SYSTEM_ERROR, CANNOT_SPAWN_VSP);
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
				throw lib::sensor::sensor_error(lib::SYSTEM_ERROR, CANNOT_LOCATE_DEVICE);
		}
	#else /* USE_MESSIP_SRR */

		VSP_NAME = config.return_attach_point_name(lib::configurator::CONFIG_SERVER, "resourceman_attach_point", _section_name);

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
		if ((pid = config.process_spawn(_section_name)) == -1)
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

	// TODO: Destruktor czujnika wirtualnego
	virtual ~sensor()
	{
		to_vsp.i_code = lib::VSP_TERMINATE;

	#if !defined(USE_MESSIP_SRR)
		if(write(sd, &to_vsp, sizeof(to_vsp)) == -1)
			sr_ecp_msg.message (lib::SYSTEM_ERROR, CANNOT_WRITE_TO_DEVICE, VSP_NAME);
		else
			close(sd);
	#else /* USE_MESSIP_SRR */
		if(messip::port_send(sd, 0, 0, to_vsp, from_vsp) < 0)
			sr_ecp_msg.message (lib::SYSTEM_ERROR, CANNOT_WRITE_TO_DEVICE, VSP_NAME);
		else
			messip::port_disconnect(sd);
	#endif /* !USE_MESSIP_SRR */

	#if defined(PROCESS_SPAWN_RSH)
		kill(pid, SIGTERM);
	#else
		SignalKill(lib::configurator::return_node_number(node_name),
				pid, 0, SIGTERM, 0, 0);
	#endif
	}

	virtual void configure_sensor(void)
	{
		to_vsp.i_code= lib::VSP_CONFIGURE_SENSOR;
	#if !defined(USE_MESSIP_SRR)
		if(write(sd, &to_vsp, sizeof(to_vsp)) != sizeof(to_vsp))
	#else /* USE_MESSIP_SRR */
		if(messip::port_send(sd, 0, 0, to_vsp, from_vsp) < 0)
	#endif /* !USE_MESSIP_SRR */
			sr_ecp_msg.message (lib::SYSTEM_ERROR, CANNOT_WRITE_TO_DEVICE, VSP_NAME);
	}

	virtual void initiate_reading(void)
	{
		to_vsp.i_code= lib::VSP_INITIATE_READING;
	#if !defined(USE_MESSIP_SRR)
		if(write(sd, &to_vsp, sizeof(to_vsp)) == -1)
	#else /* USE_MESSIP_SRR */
		if(messip::port_send(sd, 0, 0, to_vsp, from_vsp) < 0)
	#endif /* !USE_MESSIP_SRR */
			sr_ecp_msg.message (lib::SYSTEM_ERROR, CANNOT_WRITE_TO_DEVICE, VSP_NAME);
	}

	virtual void get_reading(void) {
		get_reading(image);
	}

	void get_reading(SENSOR_IMAGE & sensor_image)
	{
		// Sprawdzenie, czy uzyc domyslnego obrazu.
		to_vsp.i_code= lib::VSP_GET_READING;
	#if !defined(USE_MESSIP_SRR)
		if(read(sd, &from_vsp, sizeof(from_vsp)) == -1)
	#else /* USE_MESSIP_SRR */
		if(messip::port_send(sd, 0, 0, to_vsp, from_vsp) < 0)
	#endif /* !USE_MESSIP_SRR */
			sr_ecp_msg.message (lib::SYSTEM_ERROR, CANNOT_READ_FROM_DEVICE, VSP_NAME);

		// jesli odczyt sie powodl, przepisanie pol obrazu z bufora komunikacyjnego do image;
		if(from_vsp.vsp_report == lib::VSP_REPLY_OK) {
			image = from_vsp.comm_image;
		} else {
			sr_ecp_msg.message ("Reply from VSP not ok");
		}
	}
};

} // namespace sensor

// Kontener zawierajacy wykorzystywane czyjniki
typedef std::map<lib::SENSOR_t, sensor::sensor_interface *> sensors_t;
typedef sensors_t::value_type sensor_item_t;

} // namespace ecp_mp
} // namespace mrrocpp

#endif /* _ECP_MP_SENSOR_H */
