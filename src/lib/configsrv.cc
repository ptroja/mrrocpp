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

#include "lib/configsrv.h"

// Konstruktor obiektu - konfiguratora.
configsrv::configsrv (const char* _node, const char* _dir, const char* _ini_file)
{
	assert(_node);
	assert(_dir);
	assert(_ini_file);

	node = strdup(_node);
	dir = strdup(_dir);
	ini_file = strdup(_ini_file);

	pthread_mutex_init(&mutex, NULL );

	int size;

	size = 1 + strlen("/net/") + strlen(node) + strlen(dir);
	mrrocpp_network_path = new char[size];
	// Stworzenie sciezki do pliku.
	sprintf(mrrocpp_network_path, "/net/%s%s", node, dir);

	size = 1 + strlen(mrrocpp_network_path) + strlen("configs/") + strlen(ini_file);
	file_location = new char[size];
	// Stworzenie sciezki do pliku.
	sprintf(file_location, "%s%s%s", mrrocpp_network_path, "configs/", ini_file);

	size = 1 + strlen(mrrocpp_network_path) + strlen("configs/") + strlen("common.ini");
	common_file_location = new char[size];
	// Stworzenie sciezki do pliku.
	sprintf(common_file_location, "%s%s%s", mrrocpp_network_path, "configs/", "common.ini");
}// : configsrv


void configsrv::change_ini_file (const char* _ini_file)
{
	lock_mutex();
	if (ini_file) free(ini_file);
	ini_file = strdup(_ini_file);
	unlock_mutex();
}

int	configsrv::lock_mutex() // zajecie mutex'a
{
	return pthread_mutex_lock( &mutex );
}

int	configsrv::unlock_mutex() // zwolnienie mutex'a
{
	return pthread_mutex_unlock( &mutex );
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

	lock_mutex();
	if (input_config(file_location, configs, _section_name)<1) {
		if (input_config(common_file_location, configs, _section_name)<1) {
			unlock_mutex();

			return false;
		}
	}
	unlock_mutex();

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
	lock_mutex();
	if (input_config(file_location, configs, _section_name)<1) {
		if (input_config(common_file_location, configs, _section_name)<1) {
			printf("Blad input_config() w return_int_value file_location:%s, _section_name:%s, _key:%s\n", file_location, _section_name, _key);
		}
	}
	unlock_mutex();

// 	throw ERROR
	// Zwrocenie wartosci.

	printf("configsrv::return_int_value(\"%s\", \"%s\") = %d\n", _key, _section_name, value);

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
	lock_mutex();
	if (input_config(file_location, configs, _section_name)<1) {
		if (input_config(common_file_location, configs, _section_name)<1) {
			printf("Blad input_config() w return_double_value file_location:%s, _section_name:%s, _key:%s\n",
				file_location, _section_name, _key);
		}
	}
	unlock_mutex();

// 	throw ERROR
	// Zwrocenie wartosci.
	return value;
}// : return_int_value



// Zwraca wartosc (char*) dla klucza.
char* configsrv::return_string_value(const char* _key, const char*_section_name)
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
	lock_mutex();
	if (input_config(file_location, configs, _section_name)<1) {
		if (input_config(common_file_location, configs, _section_name)<1) {
			printf("Blad input_config() w return_string_value file_location:{%s,%s}, _section_name:%s, _key:%s\n",
				 file_location, common_file_location, _section_name, _key);
		}
	}
	unlock_mutex();
// 	throw ERROR
	// Przepisanie wartosci.
	int size = 1 + strlen(tmp);
	char * value = new char [size];
	strcpy(value, tmp);
	// Zwrocenie wartosci.
	return value;
}// : return_string_value

configsrv::~configsrv() {
	free(node);
	free(dir);
	free(ini_file);

	delete [] common_file_location;
	delete [] file_location;
}
