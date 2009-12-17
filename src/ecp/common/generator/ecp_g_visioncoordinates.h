/// \file generator/ecp_g_visioncoordinates.cc 
/// \brief generator odpowiadajacy za obliczanie wspolrzednych dla ruchu w strone
//      obiektu zaobserowanego przez system wizyjny (pobiera dane z VSP FraDIA)
/// \author Maciej Jerzy Nowak
/// \date 22.07.2009
///////////////////////////////////////////////////////////////////////////////

#ifndef ECP_G_VISIONCOORDINATES_H_INCLUDED
#define ECP_G_VISIONCOORDINATES_H_INCLUDED

#include <vector>
#include "ecp/common/task/ecp_task.h"
#include "ecp/common/generator/ecp_generator.h"
#include "lib/mrmath/mrmath.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

/// \brief klasa odpowiadajaca za generowanie nowych wspolrzednych dla koncowki robota
/// na podstawie informacji z systemu wizyjnego, z wykorzystaniem VSP FraDIA
/// odpowiada za komunikacje z FraDIA, oraz aktualnych wspolrzednych robota
class visioncoordinates : public common::generator::generator
{
public:
	/// @param _ecp_task - zadanie, z ktorym zwiazany jest dany generator
	visioncoordinates(common::task::task& _ecp_task);

    /// first step method
    virtual bool first_step();

    /// next step method
    virtual bool next_step();

	/// \brief zwraca przyblizone wspolrzedne robota, by ten blizej spojrzal na obiekt.
	/// kolejne wywolania metody zwracaja kolejne wspolrzedne (o ile zostalo rozpoznanych ich wiecej)
	/// @param [out] output - wspolrzedne w XYZ_EULER_ZYZ
	/// @return bool - true, gdy wspolrzedne sa poprawne. W przeciwnym wypadku false
	bool getCoordinates(double output[8]);

	/// \brief sprawdza z uzyciem wirtualnego czujnika, czy widzimy dany, konkretny obiekt.
	/// @return true - gdy na obrazie zostal zidentyfikowany dany obiekt, inaczej false.
	bool test();

	/// \brief ustawia nazwe obiektu, ktory bedzie poszukiwany/testowany czy jest, za pomoca VSP FraDIA
	/// \note nazwa obiektu nie jest weryfikowana (czy znajduje sie na liscie znanych FraDIA'i obiektow)
	void searchObject(const std::string& object) { itsSearchObject = object; }

	/// \brief na podstawie numeru elementu w tablicy znanych obiektow, wybiera obiekt poszukiwany za pomoca VSP FraDIA
	void searchObject(int i) { itsSearchObject = itsKnownObjects[i]; }

	/// \brief pobiera nazwy rozpoznawanych obiektow
	const std::vector<std::string>& knownObjects() const { return itsKnownObjects; }

private:
	/// \brief oblicza macierz ruchu na podstawie danych z obrazu otrzymanego z FraDIA
	/// @param rot_z - rotacji wokol osi z obrazu, by srodek poszukiwanego obiektu
	/// znalazl sie na linii pionowej w polowie obrazu
	/// @param rot_dev - odleglosc (w radianach) miedzy srodkiem poszukiwanego obiektu, a srodkiem obrazu
	/// @param distance - przyblizona odleglosc do obiektu
	/// @return macierz przeksztalcenia, by robot znalazl sie blizej obiektu i mogl sie mu przyjrzec.
	lib::Homog_matrix calculateMove(double rot_z, double rot_dev, double distance);

	/// nazwa sekcji pliku konfiguracyjnego, w ktorym przechowywane sa ustawienia
	const char* SETTINGS_SECTION_NAME;

	
	lib::SENSOR_IMAGE* sensor_in;			///< dane przychodzace z FraDIA
	lib::ECP_VSP_MSG* sensor_out;			///< dane wychodzace do FraDIA

	/// \brief prosty cache, na SIZE wspolrzednych
	template<int SIZE>
	class CoordinatesCache
	{
	public:
		CoordinatesCache() { memset(bf, 0, sizeof(bf)); }
		CoordinatesCache(const double c[SIZE]) { memcpy(bf, c, sizeof(bf)); }
		CoordinatesCache(const CoordinatesCache& rhs) { memcpy(bf, rhs.bf, sizeof(bf)); }
		CoordinatesCache& operator=(const CoordinatesCache& rhs) { memcpy(bf, rhs.bf, sizeof(bf)); return *this; }

		void to(double out[SIZE]) const { memcpy(out, bf, sizeof(bf)); }
		void from(const double in[SIZE]) const { memcpy(bf, in, sizeof(bf)); }

		double bf[SIZE];
	};

	/// cache na 8 wspolrzednych XYZ_EULER_ZYZ + gripper + (nie uzywana)
	typedef CoordinatesCache<8> EulerCoordinates;

	std::list<EulerCoordinates> itsCoordinates;	///< lista wspolrzednych, z ktorych maja byc przetestowane obiekty

	/// poszukiwany obiekt (jego nazwa)
	std::string itsSearchObject;

	/// pobiera liste nazw znanych FraDIA obiektow
	void getKnownObjects();

	/// tablica znanych obiektow
	std::vector<std::string> itsKnownObjects;
};

} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp


#endif // ECP_G_VISIONCOORDINATES_H_INCLUDED
