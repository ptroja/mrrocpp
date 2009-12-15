/// \file ecp_t_visioncoordinates.h
/// \brief definicja zadania podazania w strone zidentyfikowanego obiektu na ekranie
/// \author 2009 Maciej Jerzy Nowak
///////////////////////////////////////////////////////////////////////////////


#ifndef ECP_T_VISIONCOORDINATES_H_INCLUDED
#define ECP_T_VISIOnCOORDINATES_H_INCLUDED


#include "ecp/common/task/ecp_task.h"
#include "ecp/common/generator/ecp_g_visioncoordinates.h"
#include "ecp/common/generator/ecp_g_smooth.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace task {

	/// zadanie kierowania ramieniem robota w strone zidentyfikowanego obiektu na ekranie
	class visioncoordinates : public common::task::task
	{
	public:
		visioncoordinates(lib::configurator& _config);

		///  Main algorithm loop.
		void main_task_algorithm();

	private:
		/// ustawia poczatkowa pozycje (z kamera powyzje ramienia robota n`a podstawie pozycji zadanej w pliku konfiguracyjnym

		void setStartPosition();

		/// umozliwia uzytkownikowi wybranie obiektu do poszukiwania
		/// @return true, gdy zadanie ma byc kontynuowane
		bool selectObject();

		generator::visioncoordinates* itsVisionGen;	///< czujnik wizji
		generator::smooth* itsSmoothGen;			///< generator ruchu

		/// nazwa sekcji zawierajacej ustawienia dla zadania

		const char* SETTINGS_SECTION_NAME;
};

} // namespace task
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif	// ECP_T_VISIONCOORDINATES_H_INCLUDED
