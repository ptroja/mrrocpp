// ecp_g_visioncoordinates.cc - generator odpowiadajacy za obliczanie wspolrzednych dla ruchu w strone
//      obiektu zaobserowanego przez system wizyjny (pobiera dane z VSP FraDIA)
// (c)2009 Maciej Jerzy Nowak
// Created: 20.02.2009  Last Update: 20.02.2009
///////////////////////////////////////////////////////////////////////////////

#ifndef ECP_G_VISIONCOORDINATES_H_INCLUDED
#define ECP_G_VISIONCOORDINATES_H_INCLUDED

#include "ecp/common/ecp_task.h"
#include "ecp/common/ecp_generator.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

// class ecp_visioncoordinates_generator : public common::generator::ecp_generator
// - klasa odpowiadajaca za generowanie nowych wspolrzednych dla koncowki robota
//   na podstawie informacji z systemu wizyjnego, z wykorzystaniem VSP FraDIA
//   odpowiada za komunikacje z FraDIA, oraz aktualnych wspolrzednych robota
class visioncoordinates : public common::generator::base
{
public:
	// ecp_visioncoordinates_generator(common::task::ecp_task& _ecp_task)
	// � common::task::ecp_task& _ecp_task - zadanie, z ktorym zwiazany jest dany generator
	visioncoordinates(common::task::ecp_task& _ecp_task);

    // bool first_step()
    // - first step method
    virtual bool first_step();

    // bool next_step ()
    // - next step method
    virtual bool next_step();

	void getNewCoordinates(double output[8]) const {memcpy(output, itsOutputCoordinates, sizeof(itsOutputCoordinates)); }

private:
	// const char* SETTINGS_SECTION_NAME
	// - nazwa sekcji pliku konfiguracyjnego, w ktorym przechowywane sa ustawienia
	const char* SETTINGS_SECTION_NAME;

	// double itsOutputCoordinates[8]
	// - wyjsciowe, obliczone wspolrzedne (6 pierwszych, pozostale 2 skopiowane)
	double itsOutputCoordinates[8];

};

} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp


#endif // ECP_G_VISIONCOORDINATES_H_INCLUDED
