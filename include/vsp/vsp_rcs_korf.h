// ------------------------------------------------------------------------
// Proces:	VIRTUAL SENSOR PROCESS (VSP)
// Plik:	vsp_rcs_korf.cc
// System:	QNX/MRROC++  v. 6.3
// Opis:	Deklaracja klasy vsp_rcs_korf - znajdywanie rozwiazania algorytmem Korfa.
// Autor:	jsalacka
// Data:	25.03.2007
// ------------------------------------------------------------------------

#if !defined(_VSP_RCS_KORF_H)
#define _VSP_RCS_KORF_H

#include "vsp/vsp_sensor.h"

class CubieCube;
class CubeSolution;

// klasa czujnikow po stronie VSP
class vsp_rcs_korf : public vsp_sensor {
  private:
	// Stan kostki Rubika do rozwiazania.
	CubieCube *pCube;
	// Znalezione rozwiazanie.
	CubeSolution *pSol;
	// Blad podczas znajdywania rozwiazania
	bool bRCSErr;

  public:
	// Konstruktor czujnika wirtualnego.
	vsp_rcs_korf(void);
	// Destruktor czujnika wirtualnego.
	~vsp_rcs_korf(void);
	// Konfiguracja czujnika.
	void configure_sensor (void);
	// Inicjacja odczytu.
	void initiate_reading (void);
	// Odeslanie odczytu.
	void get_reading (void);
}; // end: class vsp_rcs_korf

#endif
