// -------------------------------------------------------------------------
// Proces: 	EFFECTOR CONTROL PROCESS (lib::ECP) 
// Plik:			configsrv.h
// System:	QNX/MRROCPP  v. 6.3
// Opis:		Plik zawiera klase configsrv - obsluga konfiguracji z pliku INI.
// Autor:		tkornuta
// Data:		10.11.2005
// -------------------------------------------------------------------------


#if !defined(_CONFIGSRV_H)
#define _CONFIGSRV_H

#include <pthread.h>
// Typy zmiennych odczytywanych z pliku INI.
#include "lib/cfgopts.h"

class configsrv
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

	char * file_location, * common_file_location;

public:

	// Konstruktor obiektu - konfiguratora.
	configsrv(const char* _node, const char* _dir, const char* _ini_file);
	
	void change_ini_file (const char* _ini_file);

	// Zwraca wartosc (int) dla klucza.
	int return_int_value(const char* _key, const char* _section_name);

	// Zwraca wartosc (double) dla klucza.
	double return_double_value(const char* _key, const char* _section_name);

	// Zwraca wartosc (char*) dla klucza.
	char* return_string_value(const char* _key, const char* _section_name);
	
	// Zwraca czy dany klucz istnieje
	bool exists(const char* _key, const char* _section_name);

	~configsrv();
	
};// : configsrv

#endif /* _CONFIGSRV_H */
