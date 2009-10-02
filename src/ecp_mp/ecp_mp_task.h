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

#include <stdint.h>
#include <map>

#include "lib/impconst.h"
#include "lib/com_buf.h"
#include "lib/srlib.h"
#include "lib/configurator.h"

#include "ecp_mp/transmitter.h"
#include "lib/sensor.h"

#include <libxml/tree.h>

#include <map>
#include "ecp_mp/Trajectory.h"

namespace mrrocpp {
namespace ecp_mp {

typedef std::map<lib::SENSOR_ENUM, lib::sensor*> sensor_map;
typedef std::map<transmitter::TRANSMITTER_ENUM, transmitter::transmitter*> transmitter_map;

namespace task {

// klasa macierzysta dla klas globalnych procesow ECP i MP
class task {

public:

	typedef std::map<const char*, ecp_mp::common::Trajectory /*, str_cmp */> trajectories_t;

	task(lib::configurator &_config);

	// mapa wszystkich czujnikow
	static std::map <lib::SENSOR_ENUM, lib::sensor*> sensor_m;

	// mapa wszystkich transmiterow
	static std::map <transmitter::TRANSMITTER_ENUM, transmitter::transmitter*> transmitter_m;

	static lib::sr_ecp* sr_ecp_msg;
	lib::configurator &config;
#if !defined(USE_MESSIP_SRR)
	int UI_fd;
#else
	messip_channel_t *UI_fd;
#endif
	std::string mrrocpp_network_path;

	// METODY
	// Odpowiedz operatora na zadane pytanie: (Yes/No)
	bool operator_reaction (const char* question );

	// by Y - Wybor przez operatora jednej z opcji
	uint8_t choose_option (const char* question, uint8_t nr_of_options_input );

	// Zadanie od operatora wprowadzenia liczby calkowitej (int)
	int input_integer (const char* question);

	// --------------------------------------------------------------------------
	// Zadanie od operatora wprowadzenia liczby rzeczywistej (double)
	double input_double (const char* question);

	// --------------------------------------------------------------------------
	// Wyswietlenie komunikatu
	bool show_message (const char* message);

	// Zatrzymanie wszystkich VSP
	static void kill_all_VSP (sensor_map & _sensor_m);

	// funkcje do obslugi czujnikow
	void all_sensors_initiate_reading (sensor_map & _sensor_m);
	void all_sensors_get_reading (sensor_map & _sensor_m);

	// funkcjonalnosc dodana na potrzeby czytania trajektorii z pliku xml
	class str_cmp{
	public:
		bool operator()(char const *a, char const *b) const;
	};

	ecp_mp::common::Trajectory * createTrajectory(xmlNodePtr actNode, xmlChar *stateID);
	trajectories_t * loadTrajectories(const char * fileName, lib::ROBOT_ENUM propRobot);
};

// ---------------------------------------------------------------
class ECP_MP_main_error
{ // Klasa obslugi bledow poziomie MP
	public:
		const lib::ERROR_CLASS error_class;
		const uint64_t mp_error;
		ECP_MP_main_error(lib::ERROR_CLASS err0, uint64_t err1, const char *file, int line) :
			error_class(err0), mp_error(err1)
		{
			fprintf(stderr, "ECP_MP_main_error @ %s:%d\n", file, line);
		}
#define ECP_MP_main_error(e0,e1)	ECP_MP_main_error((e0),(e1), __FILE__, __LINE__)
};

// ---------------------------------------------------------------

} // namespace task
} // namespace ecp_mp
} // namespace mrrocpp

#endif /* _ECP_MP_H */
