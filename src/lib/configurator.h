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

#include <boost/thread/mutex.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/tokenizer.hpp>

#if defined(USE_MESSIP_SRR)
  #include <messip.h>
#endif
// Typy zmiennych odczytywanych z pliku INI.
#include "lib/cfgopts.h"

namespace mrrocpp {
namespace lib {

#define PROCESS_SPAWN_RSH
// by Y - jesli usuna buga to mozna powrocic do tego rozwiazania ale sadze ze nie warto
//#define PROCESS_SPAWN_SPAWN

class configurator
{
private:
	const std::string node;
	const std::string dir;
	std::string ini_file;
	std::string mrrocpp_network_path;
	struct utsname sysinfo;

	// do ochrony wylacznosci dostepu do pliku miedzy watkami jednego procesu
	mutable boost::mutex file_mutex;

	const std::string session_name; // nazwa sesji skojarzona z pojedynczym uruchomieniem aplikacji mroka

#ifdef USE_MESSIP_SRR
	messip_channel_t *ch;
#else
	std::string file_location;
	std::string common_file_location;

	// Zwraca wartosc (char*) dla sciezki do pliku konfiguracyjnego.
	std::string return_ini_file_path() const;

	// Zwraca wartosc (char*) dla sciezki do pliku konfiguracyjnego.
	std::string return_common_ini_file_path() const;

#endif /* USE_MESSIP_SRR */

	// Zwraca wartosc (char*) dla klucza.
	std::string return_string_value(const char* _key, const char* __section_name = NULL) const;

public:
	std::string return_mrrocpp_network_path() const;
	std::string return_default_reader_measures_path() const;

	const std::string section_name;

    bool check_config(const std::string & s);

	// Konstruktor obiektu - konfiguratora.
	configurator(
			const std::string & _node,
			const std::string & _dir,
			const std::string & _ini_file,
			const std::string & _section_name,
			const std::string & _session_name);

	// zmiana nazwy sesji z modyfikacja pliku konfiguracyjnego
	void change_ini_file (const std::string & _ini_file);

	// Odpalenie procesu zapisanego w danej sekcji INI.
	pid_t process_spawn(const std::string & _section_name);

	// Zwraca numer wezla.
	static int return_node_number (const std::string & node_name_l);

	// Zwraca attach point'a serwerow w zaleznosci od typu

	typedef enum _config_path_type {
		CONFIG_RESOURCEMAN_LOCAL,
		CONFIG_RESOURCEMAN_GLOBAL,
		CONFIG_SERVER
	} config_path_type_t;

	std::string return_attach_point_name (config_path_type_t _type, const char* _key, const char* __section_name = NULL) const;
	std::string return_attach_point_name (config_path_type_t _type, const std::string & _key, const std::string & __section_name) const {
		return return_attach_point_name(_type, _key.c_str(), __section_name.c_str());
	};

	template<class Type>
	Type value(const std::string & _key, const std::string & __section_name) const {
		return boost::lexical_cast<Type>(return_string_value(_key.c_str(), __section_name.c_str()));
	};

	template<class Type>
	Type value(const std::string & _key) const {
		return boost::lexical_cast<Type>(return_string_value(_key.c_str()));
	};

	// Zwraca czy dany klucz istnieje
	bool exists(const char* _key, const char* __section_name = NULL) const;
	bool exists(const std::string & _key, const std::string & __section_name) const {
		return exists(_key.c_str(), __section_name.c_str());
	};

	~configurator();

};// : configurator

} // namespace lib
} // namespace mrrocpp


#endif
