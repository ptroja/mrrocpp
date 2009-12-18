// -------------------------------------------------------------------------
// Proces: 	VIRTUAL SENSOR PROCESS (lib::VSP)
// Plik:	vsp_rcs_kociemba.cc
// System:	QNX/MRROC++  v. 6.3
// Opis:	Metody rozwiazywania kostki algorytmem Kociemby - po stronie procesu VSP.
// Autor:	jsalacka
// Data:	25.03.2007
// -------------------------------------------------------------------------

#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "lib/srlib.h"

// Konfigurator
#include "lib/configurator.h"
#include "vsp/vsp_rcs_kociemba/vsp_rcs_kociemba.h"

// CubeSolver
#include "vsp/rcs/Cube/CubeTranslator.h"
#include "vsp/rcs/Kociemba/KociembaSolver.h"
#include "vsp/rcs/Kociemba/KociembaPhase1Cube.h"
#include "vsp/rcs/Kociemba/KociembaPhase2Cube.h"
#include "vsp/rcs/Kociemba/KociembaPhase1Solver.h"
#include "vsp/rcs/Kociemba/KociembaPhase2Solver.h"
#include "vsp/rcs/Kociemba/KociembaException.h"


namespace mrrocpp {
namespace vsp {
namespace sensor {

// Konstruktor czujnika wirtualnego.
rcs_kociemba::rcs_kociemba(lib::configurator &_config) : sensor(_config){
    printf("VSP KC construct\n");

	// Inicjalizuje puste pola.
	pCube = NULL;
	pSol = NULL;

	// czujnik nie jest gotowy do natychmiastowego szukania rozwiazania
	bRCSErr = true;

	// Ustawia wielkosc unii.
	union_size = sizeof(image.sensor_union.rcs);

	// Czysci odczyt.
	image.sensor_union.rcs.cube_solution[0] = '\0';

	// Ustawia brak rozwiazania optymalnego
	bRCSOpt = false;

}


// Destruktor czujnika wirtualnego.
rcs_kociemba::~rcs_kociemba(void) {
    printf("VSP KC destruct\n");

	// Zwalnia pamiec zajmowana przez struktury pomocnicze
	KociembaPhase1Cube::SClear();
	KociembaPhase2Cube::SClear();
	KociembaPhase1Solver::SClear();
	KociembaPhase2Solver::SClear();
	DataTable::ClearFolderName();

	// Czysci pola
	delete pCube;
	pCube = NULL;
	delete pSol;
	pSol = NULL;

}


// Konfiguracja czujnika.
void rcs_kociemba::configure_sensor (void){
    printf("VSP KC configure\n");

	if (to_vsp.rcs.configure_mode == lib::RCS_BUILD_TABLES) {

		// Pobiera dane z pliku konfiguracyjnego.
		const char* path = config.value<std::string>("tables_path").c_str();
		printf("VSP KC PATH = %s\n", path);

		// Konfiguruje
		DataTable::SetLog(false);
		DataTable::SetFolderName(path);

		// Wczytuje tablice przeksztalcen i odleglosci od rozwiazania
		KociembaPhase1Cube::SInit();
		KociembaPhase2Cube::SInit();
		KociembaPhase1Solver::SInit();
		KociembaPhase2Solver::SInit();

		// Ustawia czujnik jako skonfigurowany
		is_sensor_configured=true;
	}

	else if (to_vsp.rcs.configure_mode == lib::RCS_CUBE_STATE) {

		// Przygotowuje ciag znakow jako stan kostki dla FaceletCube
		char state[73] = "U:xxxxxxxxx,R:xxxxxxxxx,F:xxxxxxxxx,D:xxxxxxxxx,L:xxxxxxxxx,B:xxxxxxxxx,";
		for (int i=0; i<6; i++)
   	 		strncpy(&state[i*12+2], &to_vsp.rcs.cube_state[i*9], 9);
		printf("VSP KC STATE=%s\n", state);

		// Ustawia stan kostki Rubika do rozwiazania
		try {
			FaceletCube fCube;
			fCube.FromString(state);
			pCube = new CubieCube();
			CubeTranslator::FaceletToCubie(fCube, *pCube);
		} catch (CubeException &exp) {
		    char *err = exp.ToString();
		    printf("ERR=%s\n", err);
		    delete[] err;
			bRCSErr = true;
			throw sensor_error (lib::FATAL_ERROR, RCS_INVALID_STATE);
		}

		// Czysci znalezione wczesniej rozwiazanie, jezeli takie istnieje
		if (pSol)
			delete pSol;
		pSol = NULL;

		// czujnik gotowy do znajdywania rozwiazania - nie ma bledow
		bRCSErr = false;
	}

	// Ustawia brak gotowego odczytu.
	is_reading_ready = false;

	// nieznalezione rozwiazanie optymalne
	bRCSOpt = false;

    printf("VSP KC END configure\n");
}


// Inicjacja odczytu.
void rcs_kociemba::initiate_reading (void) {
//    printf("VSP KC initiate_reading\n");

	// nie szuka kolejnych rozwiazan po optymalnym
    if (bRCSOpt)
	    return;

	// Zglasza blad, gdy czujnik nie jest skonfigurowany.
	if(!is_sensor_configured)
		throw sensor_error (lib::FATAL_ERROR, SENSOR_NOT_CONFIGURED);

    CubeSolution *pNSol = NULL;

	// sprawdza czy moze szukac rozwiazania
	if (bRCSErr)
		return;

   // Znajduje kolejne rozwiazanie algorytmem Kociemby
	try {
		KociembaSolver kcSolver;
		if (pSol)
			pNSol = kcSolver.FindNextSolution((CubieCube*) pCube, *((KociembaSolution*) pSol));
		else
			pNSol = kcSolver.FindSolution((CubieCube*) pCube);
	} catch (KociembaException &exp) {
	    char *err = exp.ToString();
	    printf("ERR=%s\n", err);
    	delete[] err;
		bRCSErr = true;
		throw sensor_error (lib::FATAL_ERROR, RCS_EXCEPTION);
	}

	// Jezeli rozwiazanie jest krotsze od poprzedniego, zapamietuje je
	if (pNSol && (pSol == NULL || pNSol->GetLength() < pSol->GetLength())) {
		if (pSol)
		    delete pSol;
		pSol = pNSol;

		if (pSol->GetInfo(CubeSolution::INFO_OPTIMAL) || pSol->GetLength() == 0) {
			bRCSOpt = true;
			printf("VSP KC OPTIMUM\n");
		}

		// Ustawia odczyt jako gotowy
		is_reading_ready=true;
	}

}


// Oczekiwanie na wydarzenie.
void rcs_kociemba::wait_for_event(void) {
//    printf("VSP KC wait_for_event\n");

    // Odczekuje 1s jezeli znalezione juz zostalo rozwiazanie optymalne
	if (bRCSOpt || bRCSErr)
		usleep(1000*1000);
	else
		usleep(1000*100);

}


// Odeslanie odczytu.
void rcs_kociemba::get_reading (void) {
    printf("VSP KC get_reading\n");

	// Zglasza blad, gdy czujnik nie jest skonfigurowany.
	if(!is_sensor_configured)
		throw sensor_error (lib::FATAL_ERROR, SENSOR_NOT_CONFIGURED);

	// Niemozliwe znalezienie rozwiazania - wystapil blad podczas jego znajdywania
	if (	bRCSErr) {
		from_vsp.comm_image.sensor_union.rcs.reading_mode = lib::RCS_SOLUTION_NOTPOSSIBLE;

	// Ustawia brak rozwiazania, gdy odczyt nie jest gotowy.
	} else if (!is_reading_ready || pSol == NULL) {
		throw sensor_error (lib::NON_FATAL_ERROR, READING_NOT_READY);

	// Ustawia brak potrzeby rozwiazania, gdy rozwiazanie o dlugosci 0
	} else if (pSol->GetLength() == 0) {
		from_vsp.comm_image.sensor_union.rcs.reading_mode = lib::RCS_SOLUTION_NOTNEEDED;

	// Ustawia znalezione rozwiazanie do bufora
	} else {
		from_vsp.comm_image.sensor_union.rcs.reading_mode = lib::RCS_SOLUTION_FOUND;
		char *cSol = pSol->ToString();
		char solution[200];
		int j=0;
		for (int i=0; i<(int)strlen(cSol); i++) {
			if (!strchr("(),", cSol[i])) {
			    solution[j] = cSol[i];
				j++;
			}
		}
		solution[j] = '\0';
		strcpy(from_vsp.comm_image.sensor_union.rcs.cube_solution, solution);
		delete[] cSol;
		printf("VSP KC SOLUTION=%s\n", from_vsp.comm_image.sensor_union.rcs.cube_solution);
	}

	// Ustawia odczyt jako juz odczytany
	is_reading_ready=false;
}

VSP_CREATE_SENSOR(rcs_kociemba)

} // namespace sensor
} // namespace vsp
} // namespace mrrocpp

