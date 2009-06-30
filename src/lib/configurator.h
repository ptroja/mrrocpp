// -------------------------------------------------------------------------
// Proces: 	EFFECTOR CONTROL PROCESS (lib::ECP)
// Plik:			configurator.h
// System:	QNX/MRROCPP  v. 6.3
// Opis:		Plik zawiera klase lib::configurator - obsluga konfiguracji z pliku INI.
// Autor:		tkornuta
// Data:		10.11.2005
// -------------------------------------------------------------------------

#if !defined(__CONFIGURATOR_H)
#define __CONFIGURATOR_H

#include <pthread.h>
#include <sys/utsname.h>

#include <iostream>
#include <string>

// Typy zmiennych odczytywanych z pliku INI.
#include "lib/cfgopts.h"

#include "messip/messip.h"

namespace mrrocpp {
namespace lib {

#define PROCESS_SPAWN_RSH
// Z wykorzystaniem rsh w odpowiedzi na buga w qnx 6.4.0
//#define PROCESS_SPAWN_YRSH
// by Y - jesli usuna buga to mozna powrocic do tego rozwiazania ale sadze ze nie warto
//#define PROCESS_SPAWN_SPAWN

class configurator
{
private:
	char* node;
	char* dir;
	char* ini_file;
	std::string mrrocpp_network_path;
	struct utsname sysinfo;

	// do ochrony wylacznosci dostepu do pliku miedzy watkami jednego procesu
	pthread_mutex_t mutex; // = PTHREAD_MUTEX_INITIALIZER ;

	int	lock_mutex(); // zajecie mutex'a
	int	unlock_mutex(); // zwolnienie mutex'a

	char* session_name; // nazwa sesji skojarzona z pojedynczym uruchomieniem aplikacji mroka

#ifdef USE_MESSIP_SRR
	messip_channel_t *ch;
#else
	std::string file_location;
	std::string common_file_location;

	// Zwraca wartosc (char*) dla sciezki do pliku konfiguracyjnego.
	std::string return_ini_file_path();

	// Zwraca wartosc (char*) dla sciezki do pliku konfiguracyjnego.
	std::string return_common_ini_file_path();

#endif /* USE_MESSIP_SRR */

public:
	std::string return_mrrocpp_network_path();
	std::string return_default_reader_measures_path();

	char* section_name;

	// Konstruktor obiektu - konfiguratora.
	configurator(const char* _node, const char* _dir, const char* _ini_file, const char* _section_name,
		const char* _session_name);

	// zmiana nazwy sesji z modyfikacja pliku konfiguracyjnego
	void change_ini_file (const char* _ini_file);

#if defined(PROCESS_SPAWN_YRSH)
	int answer_to_y_rsh_spawn(const char* rsh_spl);
#endif
	// Odpalenie procesu zapisanego w danej sekcji INI.
	pid_t process_spawn(const char* _section_name);

	// Zwraca numer wezla.
	static int return_node_number (std::string node_name_l);

	// Zwraca attach point'a serwerow w zaleznosci od typu

	enum config_path_type {
		CONFIG_RESOURCEMAN_LOCAL,
		CONFIG_RESOURCEMAN_GLOBAL,
		CONFIG_SERVER
	};

	std::string return_attach_point_name (const int _type, const char* _key, const char* __section_name = NULL);
	std::string return_attach_point_name (const int _type, const char* _key, std::string __section_name) {
		return return_attach_point_name(_type, _key, __section_name.c_str());
	};

	// Zwraca wartosc (int) dla klucza.
	int return_int_value(const char* _key, const char* __section_name = NULL);
	int return_int_value(const char* _key, std::string __section_name) {
		return return_int_value(_key, __section_name.c_str());
	};

	// Zwraca wartosc (double) dla klucza.
	double return_double_value(const char* _key, const char* __section_name = NULL);
	double return_double_value(const char* _key, std::string __section_name) {
		return return_double_value(_key, __section_name.c_str());
	};

	// Zwraca wartosc (char*) dla klucza.
	std::string return_string_value(const char* _key, const char* __section_name = NULL);
	std::string return_string_value(const char* _key, std::string __section_name) {
		return return_string_value(_key, __section_name.c_str());
	};

	// Zwraca czy dany klucz istnieje
	bool exists(const char* _key, const char* __section_name = NULL);
	bool exists(const char* _key, std::string __section_name) {
		return exists(_key, __section_name.c_str());
	};

	~configurator();

};// : configurator

} // namespace lib
} // namespace mrrocpp


#endif
