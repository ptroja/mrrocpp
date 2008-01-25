// -------------------------------------------------------------------------
// Proces: 	VIRTUAL SENSOR PROCESS (VSP) 
// Plik:	vsp_rcs_korf.cc
// System:	QNX/MRROC++  v. 6.3
// Opis:	Metody rozwiazywania kostki algorytmem Korfa - po stronie procesu VSP.
// Autor:	jsalacka
// Data:	25.03.2007
// -------------------------------------------------------------------------

#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "common/typedefs.h"
#include "common/impconst.h"
#include "common/com_buf.h"

#include "lib/srlib.h"

// Konfigurator
#include "lib/configurator.h"
#include "vsp/vsp_rcs_korf.h"

// CubeSolver
#include "vsp/rcs/Cube/CubeTranslator.h"
#include "vsp/rcs/Korf/KorfSolver.h"
#include "vsp/rcs/Korf/KorfException.h"


// Deskryptor pliku.
int int_id;

// Zmienne konfiguracyjne.
extern configurator* config;



// Zwrocenie stworzonego obiektu - czujnika. Funkcja implementowana w plikach klas dziedziczacych.
vsp_sensor* return_created_sensor (void) {
	return new vsp_rcs_korf();
} // : return_created_sensor


// Konstruktor czujnika wirtualnego.
vsp_rcs_korf::vsp_rcs_korf(void) {
    printf("VSP KR construct\n");

	// Inicjalizuje puste pola.
	pCube = NULL;
	pSol = NULL;

	// Ustawia brak bledu
	bRCSErr = false;

	// Ustawia wielkosc unii.
	union_size = sizeof(image.rcs);

	// Ustawia czujnik jako niezainicjowany.
	is_sensor_configured = false;

	// Ustawia brak gotowego odczytu.
	is_reading_ready = false;

	// Czysci odczyt.
	image.rcs.cube_solution[0] = '\0';

};// end: vsp_rcs_korf


// Destruktor czujnika wirtualnego.
vsp_rcs_korf::~vsp_rcs_korf(void) {
    printf("VSP KR destruct\n");

	// Zwalnia pamiec zajmowana przez struktury pomocnicze
	KorfCube::SClear();
	KorfSolver::SClear();
	DataTable::ClearFolderName();

};// end: ~vsp_rcs_korf


// Konfiguracja czujnika.
void vsp_rcs_korf::configure_sensor (void){
    printf("VSP KR configure\n");

	if (to_vsp.rcs.configure_mode == RCS_BUILD_TABLES) {

		// Pobiera dane z pliku konfiguracyjnego.
		char* path = config->return_string_value("tables_path");
		printf("VSP KR PATH = %s\n", path);

		// Konfiguruje
		DataTable::SetLog(false);
		DataTable::SetFolderName(path);
		delete path;

		// Wczytuje tablice przeksztalcen i odleglosci od rozwiazania
		KorfCube::SInit();
		KorfSolver::SInit();
	}
	
	// Ustawia czujnik jako skonfigurowany
	is_sensor_configured=true;

    printf("VSP KR END configure\n");
};// end: configure_sensor


// Inicjacja odczytu.
void vsp_rcs_korf::initiate_reading (void) {
    printf("VSP KR initiate_reading\n");

	// Zglasza blad, gdy czujnik nie jest skonfigurowany.
	if(!is_sensor_configured)
		throw sensor_error (FATAL_ERROR, SENSOR_NOT_CONFIGURED);

	// Przygotowuje ciag znakow jako stan kostki dla FaceletCube
    char state[73] = "U:xxxxxxxxx,R:xxxxxxxxx,F:xxxxxxxxx,D:xxxxxxxxx,L:xxxxxxxxx,B:xxxxxxxxx,";
    for (int i=0; i<6; i++)
    	strncpy(&state[i*12+2], &to_vsp.rcs.cube_state[i*9], 9);
	printf("VSP KR STATE=%s\n", state);
	    
	// Ustawia brak bledu
	bRCSErr = false;

	// Ustawia stan kostki Rubika do rozwiazania
	try {
		FaceletCube fCube;
		fCube.FromString(state);

		pCube = new CubieCube();
		CubeTranslator::FaceletToCubie(fCube, *pCube);

		// Znajduje rozwiazanie algorytmem Korfa
		KorfSolver krSolver;
		pSol = krSolver.FindSolution(pCube);
		
	} catch (CubeException &exp) {
	    char *err = exp.ToString();
	    printf("ERR=%s\n", err);
	    delete[] err;
		bRCSErr = true;
		throw sensor_error (FATAL_ERROR, RCS_INVALID_STATE);
	} catch (KorfException &exp) {
		bRCSErr = true;
		throw sensor_error (FATAL_ERROR, RCS_EXCEPTION);
	}

	// Ustawia odczyt jako gotowy
	is_reading_ready=true;

    printf("VSP KR END initiate_reading\n");
}; // end: initiate_reading


// Odeslanie odczytu.
void vsp_rcs_korf::get_reading (void) {
    printf("VSP KR get_reading\n");

	// Zglasza blad, gdy czujnik nie jest skonfigurowany.
	if(!is_sensor_configured)
		throw sensor_error (FATAL_ERROR, SENSOR_NOT_CONFIGURED);
	
	// Niemozliwe znalezienie rozwiazania - wystapil blad podczas jego znajdywania
	if (	bRCSErr) {
		from_vsp.comm_image.rcs.reading_mode = RCS_SOLUTION_NOTPOSSIBLE;

	// Ustawia brak rozwiazania, gdy odczyt nie jest gotowy.
	} else if (!is_reading_ready || pSol == NULL) {
		throw sensor_error (NON_FATAL_ERROR, READING_NOT_READY);

	// Ustawia brak potrzeby rozwiazania, gdy rozwiazanie o dlugosci 0
	} else if (pSol->GetLength() == 0) {
		from_vsp.comm_image.rcs.reading_mode = RCS_SOLUTION_NOTNEEDED;

	// Ustawia znalezione rozwiazanie do bufora
	} else {
		from_vsp.comm_image.rcs.reading_mode = RCS_SOLUTION_FOUND;
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
		strcpy(from_vsp.comm_image.rcs.cube_solution, solution);
		delete[] cSol;
		printf("VSP KR SOLUTION=%s\n", from_vsp.comm_image.rcs.cube_solution);
	}

	// Ustawia odczyt jako juz odczytany
	is_reading_ready=false;
	
	// Czysci pola
	delete pCube;
	pCube = NULL;
	delete pSol;
	pSol = NULL;

}; // end: get_reading
