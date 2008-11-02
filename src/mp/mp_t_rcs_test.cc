// -------------------------------------------------------------------------
//                              mp_t_rcs_test.cc
// RCS - testowanie znajdywania rozwiazania kostki Rubika przy uzyciu VSP
// autor: Jadwiga Salacka
// data: 04.04.2007
// -------------------------------------------------------------------------


#include <stdio.h>
#include <unistd.h>
#include <string.h>

#include "common/typedefs.h"
#include "common/impconst.h"
#include "common/com_buf.h"

#include "lib/srlib.h"
#include "mp/mp.h"
#include "mp/mp_t_rcs_test.h"
#include "ecp_mp/ecp_mp_s_rcs_korf.h"
#include "ecp_mp/ecp_mp_s_rcs_kociemba.h"


mp_task* return_created_mp_task (configurator &_config)
{
	return new mp_task_rcs_test(_config);
}

mp_task_rcs_test::mp_task_rcs_test(configurator &_config) : mp_task(_config)
{
    printf("MP Construct\n");
};

mp_task_rcs_test::~mp_task_rcs_test()
{
    printf("MP Destruct\n");

	// konczy prace czujnikow
	sensor_m[SENSOR_RCS_KOCIEMBA]->terminate();
	sensor_m[SENSOR_RCS_KORF]->terminate();
}	
	
// methods fo mp template to redefine in concete class
void mp_task_rcs_test::task_initialization(void) 
{
    printf("MP Init\n");

	// tworzy i konfiguruje czujnik dla algorytmu Kociemby (w powloce nieinteraktywnej)
	sensor_m[SENSOR_RCS_KOCIEMBA] = new ecp_mp_rcs_kociemba(SENSOR_RCS_KOCIEMBA, "[vsp_rcs_kociemba]", *this);
	sensor_m[SENSOR_RCS_KOCIEMBA]->to_vsp.rcs.configure_mode = RCS_BUILD_TABLES;
	sensor_m[SENSOR_RCS_KOCIEMBA]->configure_sensor(); 

	// tworzy i konfiguruje czujnik dla algorytmu Korfa (w powloce interaktywnej bez oczekiwania)
	sensor_m[SENSOR_RCS_KORF] = new ecp_mp_rcs_korf(SENSOR_RCS_KORF, "[vsp_rcs_korf]", *this);
	sensor_m[SENSOR_RCS_KORF]->to_vsp.rcs.configure_mode = RCS_BUILD_TABLES;
	sensor_m[SENSOR_RCS_KORF]->configure_sensor(); 

};

// rcs - znajdz rozwiazanie
void mp_task_rcs_test::main_task_algorithm(void)
{
    printf("MP Algorithm\n");

    // Oczekiwanie na zlecenie START od UI  
	sr_ecp_msg->message("MP dla testow RCS - wcisnij start");
	wait_for_start ();
    printf("MP Start\n");


    // Testowy stan kostki (URFDLB)
    //char cube_test_state[55] = "gggggggggyyyyyyyyyrrrrrrrrrbbbbbbbbbwwwwwwwwwooooooooo";
    //char cube_test_state[55] = "gggggggggyyyyyyyyyrrrrrrrrrbbbbbbbbbwwwwwwwwowoooooooo";
    //char cube_test_state[55] = "ggggggowwgyygyygrrwrrwrrwrrryybbbbbbwwbwwboobooooooyyy";
    //char cube_test_state[55] = "gggggggbgyyyyyyyyyrrrrrrrorbbbbbbbgbwwwwwwwwwoooooooro";
    const char cube_test_state[55] = "wwggggbogyyyyyyyyywwoooowrogbbrbbwobrrogwgrworbgrrwrbb";
    //char cube_test_state[55] = "ryyggwggowrggyowyowwbwrrobbbrrbbgrwgwoobwbboyrrgyoyyoy";


	// Zmienne na rozwiazania
	char *sol = NULL, *sol_korf = NULL, *sol_kociemba = NULL;

	// petle ustawiajace stan kostki w czujnikach, az do skutku
	bool korf_configured = false;
	bool kociemba_configured = false;
	
	// konfiguruje czujnik dla algorytmu Kociemby, ten od razu rozpoczyna prace	
	sensor_m[SENSOR_RCS_KOCIEMBA]->to_vsp.rcs.configure_mode = RCS_CUBE_STATE;
	strncpy(sensor_m[SENSOR_RCS_KOCIEMBA]->to_vsp.rcs.cube_state, cube_test_state, 54);
	sensor_m[SENSOR_RCS_KOCIEMBA]->configure_sensor(); 

	// inicjuje odczyt z czujnika dla algorytmu Korfa, az do skutku
	while (!korf_configured) {
		strncpy(sensor_m[SENSOR_RCS_KORF]->to_vsp.rcs.cube_state, cube_test_state, 54);
		sensor_m[SENSOR_RCS_KORF]->initiate_reading();
		if (sensor_m[SENSOR_RCS_KORF]->image.rcs.init_mode == RCS_INIT_SUCCESS)
			korf_configured = true;
		else
			sleep(1);
	}


	// petle aktywnego czekania na rozwiazanie
	bool sol_possible = true;
	bool sol_needed = true;
	bool korf_found = false;
	bool kociemba_found = false;
	bool time_elapsed;

	// zapewnienie ze znalezione rozwiazanie Kociemby
	while (!kociemba_found && sol_possible && sol_needed) {

		time_t t1, t2;
		int timeout = config.return_int_value("korf_timeout");
		t1 = time(NULL);
		time_elapsed = false;
		while (!korf_found && !time_elapsed && sol_possible && sol_needed) {

		    // odczytuje wynik z czujnika dla algorytmu Korfa
			sensor_m[SENSOR_RCS_KORF]->get_reading();
			sleep(1);
			if (sensor_m[SENSOR_RCS_KORF]->image.rcs.reading_mode == RCS_SOLUTION_NOTPOSSIBLE) {
				sol_possible = false;
			} else if (sensor_m[SENSOR_RCS_KORF]->image.rcs.reading_mode == RCS_SOLUTION_NOTNEEDED) {
				sol_needed = false;
			} else if (sensor_m[SENSOR_RCS_KORF]->image.rcs.reading_mode == RCS_SOLUTION_FOUND) {
				sol_korf = new char[200];
				strcpy(sol_korf, (char*) sensor_m[SENSOR_RCS_KORF]->image.rcs.cube_solution);
				printf("MP KR: %s\n", sol_korf);
				korf_found = true;
			}
	
			t2 = time(NULL);
			if (t2-t1 > timeout) {
				printf("MP KR Timeot\n");
			    time_elapsed = true;
			} else {
				//printf("MP KR still searching: %d\n", (t2-t1));
			}
			
		}

		// odczytuje ostanio znalezione rozwiazanie z czujnika dla algorytmu Kociemby
		sensor_m[SENSOR_RCS_KOCIEMBA]->get_reading();
		sleep(1);
		if (sensor_m[SENSOR_RCS_KOCIEMBA]->image.rcs.reading_mode == RCS_SOLUTION_NOTPOSSIBLE) {
			sol_possible = false;
		} else if (sensor_m[SENSOR_RCS_KOCIEMBA]->image.rcs.reading_mode == RCS_SOLUTION_NOTNEEDED) {
			sol_needed = false;
		} else if (sensor_m[SENSOR_RCS_KOCIEMBA]->image.rcs.reading_mode == RCS_SOLUTION_FOUND)  {
			sol_kociemba = new char[200];
			strcpy(sol_kociemba, (char*) sensor_m[SENSOR_RCS_KOCIEMBA]->image.rcs.cube_solution);
			printf("MP KC: %s\n", sol_kociemba);
			kociemba_found = true;
		}

	}

	// sprawdza czy mozliwe i konieczne ukladanie
	if (!sol_possible)
	    printf("Rozwiazanie nie moze byc znalezione\n");
	else if (!sol_needed)
	    printf("Kostka rozwiazana\n");

	// wybiera najkrotsze rozwiazanie i informuje o nim
	else {
		int sol_len_korf = ( sol_korf == NULL ? 200 : strlen(sol_korf) );
		int sol_len_kociemba = ( sol_kociemba == NULL ? 200 : strlen(sol_kociemba) );
		if (sol_len_korf <= sol_len_kociemba) {
			sol = sol_korf;
			printf("MP KR SOL: %s\n", sol_korf);
			sol_korf = NULL;
		} else {
			sol = sol_kociemba;
			printf("MP KC SOL: %s\n", sol_kociemba);
			sol_kociemba = NULL;
		}
	
		// Usuniecie rozwiazan
		if (sol_korf) { delete[] sol_korf; sol_korf = NULL; }
		if (sol_kociemba) { delete[] sol_kociemba; sol_kociemba = NULL; }
	
	}

    printf("MP Wait for stop\n");
    
    // Oczekiwanie na STOP od UI    
    wait_for_stop (MP_THROW); // by Y - wlaczony tryb

    printf("MP Stop\n");

}
