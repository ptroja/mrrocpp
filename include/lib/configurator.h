// -------------------------------------------------------------------------
// Proces: 	EFFECTOR CONTROL PROCESS (ECP) 
// Plik:			configurator.h
// System:	QNX/MRROCPP  v. 6.3
// Opis:		Plik zawiera klase configurator - obsluga konfiguracji z pliku INI.
// Autor:		tkornuta
// Data:		10.11.2005
// -------------------------------------------------------------------------

#if !defined(__CONFIGURATOR_H)
#define __CONFIGURATOR_H

#include <pthread.h>
// Typy zmiennych odczytywanych z pliku INI.
#include "lib/cfgopts.h"

#include "messip/messip.h"

//#define PROCESS_SPAWN_RSH

class configurator
{
private:
	char* node;
	char* dir;
	char* ini_file;
	char* mrrocpp_network_path;
	
	// do ochrony wylacznosci dostepu do pliku miedzy watkami jednego procesu
	pthread_mutex_t mutex; // = PTHREAD_MUTEX_INITIALIZER ;
	
	int	lock_mutex(); // zajecie mutex'a
	int	unlock_mutex(); // zwolnienie mutex'a

	char* session_name; // nazwa sesji skojarzona z pojedynczym uruchomieniem aplikacji mroka

#ifdef USE_MESSIP_SRR
	messip_channel_t *ch;
#else
	char* file_location;
	char* common_file_location;

	// Zwraca wartosc (char*) dla sciezki do pliku konfiguracyjnego.
	char* return_ini_file_path();
	
	// Zwraca wartosc (char*) dla sciezki do pliku konfiguracyjnego.
	char* return_common_ini_file_path();
	
#endif /* USE_MESSIP_SRR */

public:

	char* return_mrrocpp_network_path();

	char* section_name;

	// Konstruktor obiektu - konfiguratora.
	configurator(const char* _node, const char* _dir, const char* _ini_file, const char* _section_name,
		const char* _session_name);
	
	// zmiana nazwy sesji z modyfikacja pliku konfiguracyjnego
	void change_ini_file (const char* _ini_file);

	char* return_default_reader_measures_path();
	
	// Odpalenie procesu zapisanego w danej sekcji INI.
	pid_t process_spawn_old(const char* _section_name);
	// Odpalenie procesu zapisanego w danej sekcji INI. RSH
	pid_t process_spawn(const char* _section_name);

	// Zwraca numer wezla.
	static int return_node_number (const char* node_name_l);

	// Zwraca attach point'a serwerow w zaleznosci od typu
	
	enum config_path_type {
		CONFIG_RESOURCEMAN_LOCAL,
		CONFIG_RESOURCEMAN_GLOBAL,
		CONFIG_SERVER
	};
	
	char* return_attach_point_name (const int _type, const char* _key, const char* __section_name = NULL);
	
	// Zwraca wartosc (int) dla klucza.
	int return_int_value(const char* _key, const char* __section_name = NULL);

	// Zwraca wartosc (double) dla klucza.
	double return_double_value(const char* _key, const char* __section_name = NULL);

	// Zwraca wartosc (char*) dla klucza.
	char* return_string_value(const char* _key, const char* __section_name = NULL);
	
	// Zwraca czy dany klucz istnieje
	bool exists(const char* _key, const char* __section_name = NULL);

	~configurator();
	
};// : configurator


#endif
