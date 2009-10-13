// -------------------------------------------------------------------------
// Plik:			configsrv.cc
// System:	QNX/MRROCPP  v. 6.3
// Opis:		Plik zawiera definicje matod klasy configsrv - obsluga konfiguracji z pliku INI.
// Autor:		ptrojane
// Data:		10.11.2005
// -------------------------------------------------------------------------

#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <stdlib.h>

#include <unistd.h>
#include <sys/wait.h>

#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>

#include "lib/configsrv.h"

// Konstruktor obiektu - konfiguratora.
configsrv::configsrv (const char* _node, const char* _dir, const char* _ini_file)
	: node(_node), dir(_dir), ini_file(_ini_file)
{
	// Stworzenie sciezki do pliku.
	mrrocpp_network_path = "/net/";
	mrrocpp_network_path += node;
	mrrocpp_network_path += dir;

	// Stworzenie sciezki do pliku.
	file_location = mrrocpp_network_path;
	file_location += "configs/";
	file_location += ini_file;

	// Stworzenie sciezki do pliku.
	common_file_location = mrrocpp_network_path;
	common_file_location += "configs/";
	common_file_location += "common.ini";
}// : configsrv

void configsrv::change_ini_file (const char* _ini_file)
{
	boost::mutex::scoped_lock l(mtx);
	ini_file = _ini_file;
}

// Zwraca czy dany klucz istnieje
bool configsrv::exists(const char* _key, const char* _section_name)
{
	int value;
	struct Config_Tag configs[] = {
		// Pobierane pole.
		{ (char *) _key, Int_Tag, &value},
		// Pole konczace.
		{ NULL , Error_Tag, NULL }
	};

	boost::mutex::scoped_lock l(mtx);
	if (input_config(file_location, configs, _section_name)<1) {
		if (input_config(common_file_location, configs, _section_name)<1) {
			return false;
		}
	}

	return true;
}



// Zwraca wartosc (int) dla klucza.
int configsrv::return_int_value(const char* _key, const char* _section_name)
{
	// Zwracana zmienna.
	int value = 0;
	struct Config_Tag configs[] = {
		// Pobierane pole.
		{ (char *) _key, Int_Tag, &value},
		// Pole konczace.
		{ NULL , Error_Tag, NULL }
		};

	// Odczytanie zmiennej.
	boost::mutex::scoped_lock l(mtx);
	if (input_config(file_location, configs, _section_name)<1) {
		if (input_config(common_file_location, configs, _section_name)<1) {
			fprintf(stderr, "Blad input_config() w return_int_value file_location: %s, _section_name:%s, _key:%s\n",
					file_location.c_str(), _section_name, _key);
		}
	}

// 	TODO: throw ERROR

	// Zwrocenie wartosci.
	return value;
}// : return_int_value


// Zwraca wartosc (double) dla klucza.
double configsrv::return_double_value(const char* _key, const char*_section_name)
{
	// Zwracana zmienna.
	double value = 0;
	struct Config_Tag configs[] = {
		// Pobierane pole.
		{ (char *) _key, Double_Tag, &value},
		// Pole konczace.
		{ NULL , Error_Tag, NULL }
		};

	// Odczytanie zmiennej.
	boost::mutex::scoped_lock l(mtx);
	if (input_config(file_location, configs, _section_name)<1) {
		if (input_config(common_file_location, configs, _section_name)<1) {
			printf("Blad input_config() w return_double_value file_location:%s, _section_name:%s, _key:%s\n",
				file_location.c_str(), _section_name, _key);
		}
	}

// 	TODO: throw ERROR

	// Zwrocenie wartosci.
	return value;
}// : return_int_value



// Zwraca wartosc (char*) dla klucza.
std::string configsrv::return_string_value(const char* _key, const char*_section_name)
{
	// Zwracana zmienna.
	char tmp[200];
	struct Config_Tag configs[] = {
		// Pobierane pole.
		{ (char *) _key, String_Tag, tmp},
		// Pole konczace.
		{ NULL , Error_Tag, NULL }
		};

	// Odczytanie zmiennej.
	boost::mutex::scoped_lock l(mtx);
	if (input_config(file_location, configs, _section_name)<1) {
		if (input_config(common_file_location, configs, _section_name)<1) {
			printf("Blad input_config() w return_string_value file_location:{%s,%s}, _section_name:%s, _key:%s\n",
				 file_location.c_str(), common_file_location.c_str(), _section_name, _key);
		}
	}

// 	TODO: throw ERROR

	// Zwrocenie wartosci.
	return std::string(tmp);
}// : return_string_value
