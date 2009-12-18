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

#include <boost/thread/mutex.hpp>
#include <boost/lexical_cast.hpp>

class configsrv
{
private:
	const std::string node, dir;
	std::string ini_file, mrrocpp_network_path;
	std::string file_location, common_file_location;

	// do ochrony wylacznosci dostepu do pliku miedzy watkami jednego procesu
	boost::mutex mtx;

	// Zwraca wartosc dla klucza.
	std::string return_string_value(const std::string & _key, const std::string & _section_name);

public:
	// Konstruktor obiektu - konfiguratora.
	configsrv(const std::string & _node, const std::string & _dir, const std::string & _ini_file);

	void change_ini_file (const std::string & _ini_file);

	// Zwraca wartosc dla klucza.
	template <class Type>
	Type value(const std::string & _key, const std::string & _section_name) {
		return boost::lexical_cast<Type>(return_string_value(_key, _section_name));
	}

	// Zwraca czy dany klucz istnieje
	bool exists(const std::string & _key, const std::string & _section_name);
};// : configsrv

#endif /* _CONFIGSRV_H */
