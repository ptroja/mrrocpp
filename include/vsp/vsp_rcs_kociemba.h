// ------------------------------------------------------------------------
// Proces:	VIRTUAL SENSOR PROCESS (VSP)
// Plik:	vsp_rcs_korf.cc
// System:	QNX/MRROC++  v. 6.3
// Opis:	Deklaracja klasy vsp_rcs_kociemba - znajdywanie rozwiazania algorytmem Kociemby.
// Autor:	jsalacka
// Data:	25.03.2007
// ------------------------------------------------------------------------

#if !defined(_VSP_RCS_KOCIEMBA_H)
#define _VSP_RCS_KOCIEMBA_H

#include "vsp/vsp_sensor.h"

class CubieCube;
class CubeSolution;

namespace mrrocpp {
namespace vsp {
namespace sensor {



// klasa czujnikow po stronie VSP
class vsp_rcs_kociemba : public vsp_sensor {
  private:
	// Stan kostki Rubika do rozwiazania.
	CubieCube *pCube;
	// Znalezione rozwiazanie.
	CubeSolution *pSol;
	// Znacznik znalezienie rozwiazania optymalnego
	bool bRCSOpt;
	// Blad podczas znajdywania rozwiazania
	bool bRCSErr;

  public:
	// Konstruktor czujnika wirtualnego.
	vsp_rcs_kociemba(configurator &_config);
	// Destruktor czujnika wirtualnego.
	~vsp_rcs_kociemba(void);
	// Konfiguracja czujnika.
	void configure_sensor (void);
	// Inicjacja odczytu.
	void initiate_reading (void);
	// Oczekiwanie na wydarzenie.
	void wait_for_event(void);
	// Odeslanie odczytu.
	void get_reading (void);
}; // end: class vsp_rcs_korf

} // namespace sensor
} // namespace vsp
} // namespace mrrocpp

#endif
