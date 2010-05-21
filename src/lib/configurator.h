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
#include <stdexcept>

#include <boost/thread/mutex.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/tokenizer.hpp>
#include <boost/algorithm/string/trim.hpp>
#include <boost/algorithm/string/classification.hpp>

#include <Eigen/Core>

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
			configurator(const std::string & _node, const std::string & _dir, const std::string & _ini_file, const std::string & _section_name, const std::string & _session_name);

	// zmiana nazwy sesji z modyfikacja pliku konfiguracyjnego
	void change_ini_file(const std::string & _ini_file);

	// Odpalenie procesu zapisanego w danej sekcji INI.
	pid_t process_spawn(const std::string & _section_name);

	// Zwraca numer wezla.
	static int return_node_number(const std::string & node_name_l);

	// Zwraca attach point'a serwerow w zaleznosci od typu

	typedef enum _config_path_type
	{
		CONFIG_RESOURCEMAN_LOCAL, CONFIG_RESOURCEMAN_GLOBAL, CONFIG_SERVER
	} config_path_type_t;

	std::string
			return_attach_point_name(config_path_type_t _type, const char* _key, const char* __section_name = NULL) const;
	std::string return_attach_point_name(config_path_type_t _type, const std::string & _key, const std::string & __section_name) const
	{
		return return_attach_point_name(_type, _key.c_str(), __section_name.c_str());
	}
	;

	template <class Type>
	Type value(const std::string & _key, const std::string & __section_name) const
	{
		std::string str_value = return_string_value(_key.c_str(), __section_name.c_str());
		boost::algorithm::trim(str_value);
		try{
			return boost::lexical_cast <Type>(str_value);
		}catch(const std::exception &ex){
			throw std::runtime_error("lib::configurator::value() section \"" + __section_name + "\", key: \"" + _key + "\", value: \"" + str_value + "\", " + ex.what());
		}
	}

	template <class Type>
	Type value(const std::string & _key) const
	{
		std::string str_value =return_string_value(_key.c_str());
		boost::algorithm::trim(str_value);
		try{
			return boost::lexical_cast <Type>(str_value);
		}catch(const std::exception &ex){
			throw std::runtime_error("lib::configurator::value() key: \"" + _key + "\", value: \"" + str_value + "\", " + ex.what());
		}
	}

	//	/**
	//	 * Read vector from config. Vector has format similar to MatLAB, for example: [ x y z ].
	//	 * @param name
	//	 * @param n vector size
	//	 * @return vector read
	//	 * @throws exception if vector has not been read
	//	 */
	//	boost::numeric::ublas::vector <double> value(const std::string & key, const std::string & section_name, int n) const;
	//
	//	/**
	//	 * Read matrix from config. Matrix has format similar to MatLAB, for example: [ a b c d; e f g h ].
	//	 * @param name
	//	 * @param n matrix size - rows
	//	 * @param m matrix size - columns
	//	 * @return vector read
	//	 * @throws exception if vector has not been read
	//	 */
	//	boost::numeric::ublas::matrix <double>
	//			value(const std::string & key, const std::string & section_name, int n, int m) const;

	template <int ROWS, int COLS>
	Eigen::Matrix <double, ROWS, COLS> value(const std::string & key, const std::string & section_name) const;

	// Zwraca czy dany klucz istnieje
	bool exists(const char* _key, const char* __section_name = NULL) const;
	bool exists(const std::string & _key, const std::string & __section_name) const
	{
		return exists(_key.c_str(), __section_name.c_str());
	}

	~configurator();

protected:
	//	/**
	//	 * Extract elements from vector or matrix row. For example: " 1   2 3   4 "
	//	 */
	//	boost::numeric::ublas::vector <double> get_vector_elements(std::string text_value, int n) const;

	/**
	 * Extract elements from matrix row. For example: string " 1   2 3   4 " will be interpreted as a vector {1,2,3,4}
	 */
	template <int COLS>
	Eigen::Matrix <double, 1, COLS> get_vector_elements(std::string text_value) const;

};// : configurator

template <int COLS>
Eigen::Matrix <double, 1, COLS> configurator::get_vector_elements(std::string text_value) const
{
//	std::cout << "configurator::get_vector_elements() begin\n";
	Eigen::Matrix <double, 1, COLS> row_value;
	const char * blank_chars = { " \t\n\r" };
	boost::algorithm::trim_if(text_value, boost::algorithm::is_any_of(blank_chars));

	// split it into elements
	boost::char_separator <char> space_separator(blank_chars);
	boost::tokenizer <boost::char_separator <char> > tok(text_value, space_separator);

	int element_no = 0;
	for (boost::tokenizer <boost::char_separator <char> >::iterator it = tok.begin(); it != tok.end(); ++it, ++element_no) {
		//std::cout << " " << *it << ", ";
		if (element_no >= COLS) {
			throw std::logic_error("configurator::get_vector_elements(): vector has more elements than expected: \"" + text_value + "\".");
		}
		std::string element = *it;
		boost::algorithm::trim(element);
		double element_value = boost::lexical_cast <double>(element);
		row_value(0, element_no) = element_value;
	}

	if (element_no != COLS) {
		throw std::logic_error("configurator::get_vector_elements(): vector has less elements than expected: \"" + text_value + "\".");
	}
//	std::cout << "configurator::get_vector_elements() end\n";
	return row_value;
}

template <int ROWS, int COLS>
Eigen::Matrix <double, ROWS, COLS> configurator::value(const std::string & key, const std::string & section_name) const
{
//	std::cout << "configurator::get_matrix_value() begin\n";
	Eigen::Matrix <double, ROWS, COLS> matrix_value;

	// get string value and remove leading and trailing spaces
	std::string text_value = return_string_value(key.c_str(), section_name.c_str());
	boost::algorithm::trim(text_value);

	//std::cout << "visual_servo_regulator::get_matrix_value() Processing value: "<<text_value<<"\n";

	// check for [ and ], and then remove it
	if (text_value.size() < 3 || text_value[0] != '[' || text_value[text_value.size() - 1] != ']') {
		throw std::logic_error("configurator::value(): leading or trailing chars [] not found or no value supplied for parameter: \"" + key + "\" in section \"" + section_name + "\"");
	}
	boost::algorithm::trim_if(text_value, boost::algorithm::is_any_of("[]"));

	boost::char_separator <char> semicolon_separator(";");
	boost::tokenizer <boost::char_separator <char> > tok(text_value, semicolon_separator);

	int row_no = 0;
	for (boost::tokenizer <boost::char_separator <char> >::iterator it = tok.begin(); it != tok.end(); ++it, ++row_no) {
		if (row_no >= ROWS) {
			throw std::logic_error("configurator::value(): matrix has more rows than expected. Parameter: \"" + key + "\" in section \"" + section_name + "\"");
		}
		matrix_value.row(row_no) = get_vector_elements<COLS>(*it);
	}
	if (row_no != ROWS) {
		throw std::logic_error("configurator::value(): matrix has more rows than expected. Parameter: \"" + key + "\" in section \"" + section_name + "\"");
	}

	return matrix_value;
}



} // namespace lib
} // namespace mrrocpp


#endif
