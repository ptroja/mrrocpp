// -------------------------------------------------------------------------
//                                   transformer_error.h
// Definicje struktur danych i metod dla procesu EDP
//
// Ostatnia modyfikacja: 16.04.98
// -------------------------------------------------------------------------

#ifndef __TRANSFORMER_ERROR_H
#define __TRANSFORMER_ERROR_H

#include <stdint.h>

namespace mrrocpp {
namespace edp {
namespace common {
namespace exception {

// Klasa bledow systemowych zawiazanych z komunikacja miedzyprocesowa
class System_error
{

};


class Fatal_error
{ // klasa wyjatku obslugujacego bledy fatalne
	public:
		const uint64_t error0; // Blad powstaly w servomechanizmie
		const uint64_t error1; // Blad powstaly w servomechanizmie
		Fatal_error(uint64_t err_no_0, uint64_t err_no_1);
		// przekazywanego procedurze obslugi wyjatku
};

class NonFatal_error_1
{ // klasa wyjatku obslugujacego bledy, ktore nie sa fatalne, a naleza do pierwszej grupy
	public:
		const uint64_t error; // Blad powstaly przy przeliczaniu wspolrzednych
		NonFatal_error_1(uint64_t err_no);
		// przekazywanego procedurze obslugi wyjatku
};

class NonFatal_error_2
{ // klasa wyjatku obslugujacego bledy, ktore nie sa fatalne, a naleza do drugiej grupy
	public:
		const uint64_t error; // Blad
		NonFatal_error_2(uint64_t err_no);
		// przekazywanego procedurze obslugi wyjatku
};

class NonFatal_error_3
{ // klasa wyjatku obslugujacego bledy, ktore nie sa fatalne, a naleza do drugiej grupy
	public:
		const uint64_t error; // Blad powstaly przy przeliczaniu wspolrzednych
		NonFatal_error_3(uint64_t err_no);
		// przekazywanego procedurze obslugi wyjatku
};

class NonFatal_error_4
{ // klasa wyjatku obslugujacego bledy, ktore nie sa fatalne
	public:
		const uint64_t error; // Blad
		NonFatal_error_4(uint64_t err_no);
		// przekazywanego procedurze obslugi wyjatku
};

} // namespace exception
} // namespace common
} // namespace kinematic
} // namespace mrrocpp

#endif
