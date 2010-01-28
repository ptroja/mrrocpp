// -------------------------------------------------------------------------
// Plik:			configsrv.cc
// System:	QNX/MRROCPP  v. 6.3
// Opis:		Plik zawiera definicje matod klasy configsrv - obsluga konfiguracji z pliku INI.
// Autor:		ptrojane
// Data:		10.11.2005
// -------------------------------------------------------------------------

#include <stdio.h>

#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>

#include "lib/configsrv.h"
#include "lib/cfgopts.h"

// Konstruktor obiektu - konfiguratora.
configsrv::configsrv (const std::string & _node, const std::string & _dir, const std::string & _ini_file)
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

void configsrv::change_ini_file (const std::string & _ini_file)
{
	boost::mutex::scoped_lock l(mtx);
	ini_file = _ini_file;
}

bool configsrv::exists(const std::string & _key, const std::string & _section_name)
{
	int value;
	struct Config_Tag configs[] = {
		// Pobierane pole.
		{ (char *) _key.c_str(), Int_Tag, &value},
		// Pole konczace.
		{ NULL , Error_Tag, NULL }
	};

	boost::mutex::scoped_lock l(mtx);
	if (input_config(file_location, configs, _section_name.c_str())<1) {
		if (input_config(common_file_location, configs, _section_name.c_str())<1) {
			return false;
		}
	}

	return true;
}

std::string configsrv::return_string_value(const std::string & _key, const std::string &_section_name)
{
	// Zwracana zmienna.
	char tmp[200];
	struct Config_Tag configs[] = {
		// Pobierane pole.
		{ (char *) _key.c_str(), String_Tag, tmp},
		// Pole konczace.
		{ NULL , Error_Tag, NULL }
		};

	// Odczytanie zmiennej.
	boost::mutex::scoped_lock l(mtx);
	if (input_config(file_location, configs, _section_name.c_str())<1) {
		if (input_config(common_file_location, configs, _section_name.c_str())<1) {
			printf("Blad input_config() w return_string_value file_location:{%s,%s}, _section_name:%s, _key:%s\n",
				 file_location.c_str(), common_file_location.c_str(), _section_name.c_str(), _key.c_str());
		}
	}

// 	TODO: throw ERROR

	// Zwrocenie wartosci.
	return std::string(tmp);
}// : return_string_value
