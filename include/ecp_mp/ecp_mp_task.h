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

#include "common/impconst.h"
#include "common/com_buf.h"
#include "lib/srlib.h"
#include "lib/configurator.h"

#include "ecp_mp/transmitter.h"
#include "common/sensor.h"

// klasa macierzysta dla klas globalnych procesow ECP i MP
class ecp_mp_task {

public:
	
	ecp_mp_task(configurator &_config);

	// mapa wszystkich czujnikow
	static std::map <SENSOR_ENUM, sensor*> sensor_m;
	
	// mapa wszystkich transmiterow
	static std::map <TRANSMITTER_ENUM, transmitter*> transmitter_m;

	static sr_ecp* sr_ecp_msg;
	configurator &config;

	int UI_fd;
	char* mrrocpp_network_path;
	
	// METODY
	// Odpowiedz operatora na zadane pytanie: (Yes/No)
	bool operator_reaction (const char* question );
	
	// by Y - Wybor przez operatora jednej z opcji
	BYTE choose_option (const char* question, BYTE nr_of_options_input );
	
	// Zadanie od operatora wprowadzenia liczby calkowitej (int)
	int input_integer (const char* question);
	
	// --------------------------------------------------------------------------
	// Zadanie od operatora wprowadzenia liczby rzeczywistej (double)
	double input_double (const char* question);
	
	// --------------------------------------------------------------------------
	// Wyswietlenie komunikatu
	bool show_message (const char* message);
	
	// Zatrzymanie wszystkich VSP
	static void kill_all_VSP (std::map <SENSOR_ENUM, sensor*>& _sensor_m);
	
	// funkcje do obslugi czujnikow
	void all_sensors_initiate_reading (std::map <SENSOR_ENUM, sensor*>& _sensor_m);
	void all_sensors_get_reading (std::map <SENSOR_ENUM, sensor*>& _sensor_m);
	  
};

// ---------------------------------------------------------------
class ECP_MP_main_error {  // Klasa obslugi bledow poziomie MP
  public:
    uint64_t error_class; 
    uint64_t mp_error;
    ECP_MP_main_error (uint64_t err0, uint64_t err1) { error_class = err0; mp_error = err1; }
};
// ---------------------------------------------------------------

#endif /* _ECP_MP_H */
