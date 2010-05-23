// -------------------------------------------------------------------------
//                            ecp_mp_task.h dla QNX6
// Definicje wspolnych struktur danych i metod dla procesow ECP i MP
// w szczegolnosci zwiazanych z obsluga czujnikow wirtualnych
//
// Ostatnia modyfikacja: 03.11.2005
// autor modyfikacji: tkornuta
// -------------------------------------------------------------------------

#if !defined(_ECP_MP_H)
#define _ECP_MP_H

#include "lib/agent/Agent.h"

#include <map>

#include <stdint.h>
#include <libxml/tree.h>

#include "lib/impconst.h"
#include "lib/com_buf.h"
#include "lib/srlib.h"
#include "lib/configurator.h"

#include "ecp_mp/transmitter/transmitter.h"
#include "ecp_mp/sensor/ecp_mp_sensor.h"

#include "ecp_mp/Trajectory.h"

namespace mrrocpp {
namespace ecp_mp {
namespace task {

// klasa macierzysta dla klas globalnych procesow ECP i MP
class task : Agent
{
public:
	typedef std::map <const char *, ecp_mp::common::Trajectory /*, str_cmp */> trajectories_t;

	task(lib::configurator &_config);
	virtual ~task();

	// mapa wszystkich czujnikow
	sensors_t sensor_m;

	// mapa wszystkich transmiterow
	transmitters_t transmitter_m;

	static lib::sr_ecp* sr_ecp_msg; // TODO: rename from _ecp_ (?!)
	static lib::sr_ecp* sh_msg; // TODO: rename from _ecp_ (?!)

	lib::configurator &config;

#if !defined(USE_MESSIP_SRR)
	int UI_fd;
#else
	messip_channel_t *UI_fd;
#endif
	const std::string mrrocpp_network_path;

	// METODY
	// Odpowiedz operatora na zadane pytanie: (Yes/No)
	bool operator_reaction(const char* question);

	// by Y - Wybor przez operatora jednej z opcji
	uint8_t choose_option(const char* question, uint8_t nr_of_options_input);

	// Zadanie od operatora wprowadzenia liczby calkowitej (int)
	int input_integer(const char* question);

	// Zadanie od operatora wprowadzenia liczby rzeczywistej (double)
	double input_double(const char* question);

	// Wyswietlenie komunikatu
	bool show_message(const char* message);

	// funkcje do obslugi czujnikow
	void all_sensors_initiate_reading(sensors_t & _sensor_m);
	void all_sensors_get_reading(sensors_t & _sensor_m);

	// methods for ECP template to redefine in concrete classes
	virtual void main_task_algorithm(void) = 0;

	bool step(void) {
		main_task_algorithm();
		return true;
	}

	// funkcjonalnosc dodana na potrzeby czytania trajektorii z pliku xml
	class str_cmp
	{
	public:
		bool operator()(char const *a, char const *b) const;
	};

	//ecp_mp::common::Trajectory * createTrajectory(xmlNodePtr actNode, xmlChar *stateID);
	ecp_mp::common::Trajectory * createTrajectory2(xmlNodePtr actNode, xmlChar *stateID);
	trajectories_t * loadTrajectories(const char * fileName, lib::robot_name_t propRobot);
};

} // namespace task
} // namespace ecp_mp
} // namespace mrrocpp

#endif /* _ECP_MP_H */
