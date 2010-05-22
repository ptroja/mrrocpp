// -------------------------------------------------------------------------
//                                   transformer_error.h
// Definicje struktur danych i metod dla procesu EDP
//
// Ostatnia modyfikacja: 16.04.98
// -------------------------------------------------------------------------

#ifndef __TRANSFORMER_ERROR_H
#define __TRANSFORMER_ERROR_H

#include <boost/throw_exception.hpp>
#include <boost/exception/exception.hpp>
#include <boost/exception/info.hpp>
#include <stdexcept>

#include <stdint.h>

namespace mrrocpp {
namespace lib {
namespace exception {

typedef boost::error_info<struct tag_error, uint64_t> error_code;
typedef boost::error_info<struct tag_h_errno, int> h_errno_code;

typedef boost::error_info<struct tag_err0, uint64_t> err0;
typedef boost::error_info<struct tag_err1, uint64_t> err1;

class Error_base : public virtual boost::exception, public virtual std::exception
{
public:
	~Error_base() throw ();
};

// Klasa bledow systemowych zawiazanych z komunikacja miedzyprocesowa
class System_error : public Error_base
{
public:
	~System_error() throw ();
};

// klasa wyjatku obslugujacego bledy fatalne
class Fatal_error : public Error_base
{
public:
	~Fatal_error() throw ();
};

class NonFatal_error : public Error_base
{
public:
	NonFatal_error();
	~NonFatal_error() throw ();
};

class NonFatal_error_1 : NonFatal_error
{ // klasa wyjatku obslugujacego bledy, ktore nie sa fatalne, a naleza do pierwszej grupy
public:
	const uint64_t error; // Blad powstaly przy przeliczaniu wspolrzednych
	NonFatal_error_1(uint64_t err_no);
	// przekazywanego procedurze obslugi wyjatku
};

class NonFatal_error_2 : NonFatal_error
{ // klasa wyjatku obslugujacego bledy, ktore nie sa fatalne, a naleza do drugiej grupy
public:
	const uint64_t error; // Blad
	NonFatal_error_2(uint64_t err_no);
	// przekazywanego procedurze obslugi wyjatku
};

class NonFatal_error_3 : NonFatal_error
{ // klasa wyjatku obslugujacego bledy, ktore nie sa fatalne, a naleza do drugiej grupy
public:
	const uint64_t error; // Blad powstaly przy przeliczaniu wspolrzednych
	NonFatal_error_3(uint64_t err_no);
	// przekazywanego procedurze obslugi wyjatku
};

class NonFatal_error_4 : NonFatal_error
{ // klasa wyjatku obslugujacego bledy, ktore nie sa fatalne
public:
	const uint64_t error; // Blad
	NonFatal_error_4(uint64_t err_no);
	// przekazywanego procedurze obslugi wyjatku
};

} // namespace exception
} // namespace common
} // namespace mrrocpp

#endif
