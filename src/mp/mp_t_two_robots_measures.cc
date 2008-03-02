// ------------------------------------------------------------------------
// Proces:		MASTER PROCESS (MP)
// Plik:			mp_t_two_robots_measures.cc
// System:	QNX/MRROC++  v. 6.3
// Opis:		Zadanie odpowiedzalne za zbieranie pomiarow
//				do weryfikacji kalibracji
//				- definicja metod klasy
//
// Autor:		tkornuta
// Data:		23.02.2007
// ------------------------------------------------------------------------

#include "mp/mp_t_two_robots_measures.h"
#include "mp/mp_g_two_robots_measures.h"

// Zwrocenie obiektu - zadania.
mp_task* return_created_mp_task(configurator &_config)
{
	return new mp_two_robots_measures_task(_config);
}

mp_two_robots_measures_task::mp_two_robots_measures_task(configurator &_config)
: mp_task(_config)
{
}
	
// Inicjalizacja obiektow uzywanych prez zadanie.
void mp_two_robots_measures_task::task_initialization(void) 
{
	// Stworzenie generatora.
	rmg = new mp_two_robots_measures_generator(*this);
	rmg->robot_m = robot_m;

	sr_ecp_msg->message("Two robots measurements task is ready for work.");
}

// Wlasciwe zadanie.
void mp_two_robots_measures_task::main_task_algorithm(void)
{
	// Oczekiwanie na polecenie START od UI.
	wait_for_start ();
	// Wyslanie START do wszystkich procesow ECP.
	start_all (robot_m);
	
	sr_ecp_msg->message("Started!!");
	Move(*rmg);
	sr_ecp_msg->message("Po move");

	// Oczekiwanie na STOP od UI.
	wait_for_stop (MP_THROW);

	// Wyslanie STOP do wszystkich ECP po zakonczeniu programu uzytkownika.
	terminate_all (robot_m);
}
