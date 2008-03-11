// ------------------------------------------------------------------------
// Proces:		MASTER PROCESS (MP)
// Plik:			mp_g_two_robots_measures.cc
// System:	QNX/MRROC++  v. 6.3
// Opis:		Generator odpowiedzalny za zbieranie pomiarow
//				do weryfikacji kalibracji
//				- definicja metod klasy
//
// Autor:		tkornuta
// Data:		28.02.2007
// ------------------------------------------------------------------------

#include <string.h>
#include <math.h>
#include <errno.h>
#include <unistd.h>
#include <iostream>
#include <fstream>

#include "mp/mp_g_two_robots_measures.h"

// Konstruktor.
mp_two_robots_measures_generator::mp_two_robots_measures_generator(mp_task& _mp_task)
	: mp_generator (_mp_task), UI_fd(_mp_task.UI_fd)
{
};//: mp_two_robots_measures_generator

// Pierwszy krok generatora.
bool mp_two_robots_measures_generator::first_step()
{
	idle_step_counter = 2;
	// Ustawienie polecen dla robota na torze.
	irp6ot = robot_m[ROBOT_IRP6_ON_TRACK];
	irp6ot->ecp_td.mp_command = NEXT_POSE; 
	irp6ot->ecp_td.instruction_type = GET;
	irp6ot->ecp_td.get_type = ARM_DV;
	irp6ot->ecp_td.get_arm_type = XYZ_EULER_ZYZ;
	irp6ot->ecp_td.motion_type = ABSOLUTE;
	// Ustawienie polecen dla robota na postumencie.
	irp6p = robot_m[ROBOT_IRP6_POSTUMENT];
	irp6p->ecp_td.mp_command = NEXT_POSE; 
	irp6p->ecp_td.instruction_type = GET;
	irp6p->ecp_td.get_type = ARM_DV;
	irp6p->ecp_td.get_arm_type = XYZ_EULER_ZYZ;
	irp6p->ecp_td.motion_type = ABSOLUTE;
	// Przepisanie polecen.

	
	// Wyczyszczenie listy.
	measures.clear();
	// Wyzerowanie ostatniego odczytu.
	for(int i=0; i<6; i++)
		last_measure.irp6ot[i] = 0.0;
	for(int i=0; i<6; i++)
		last_measure.irp6p[i] = 0.0;
	return true;
};// first_step()


// Nastepny krok generatora.
bool mp_two_robots_measures_generator::next_step()
{
	// Sprawdzenie, czy nadeszlo polecenie zakonczenia zbierania pomiarow.
	if (check_and_null_trigger()) 
	{
		std::cout<<"Liczba zebranych pozycji: "<<measures.size()<<std::endl;
		// Zresetowanie przerwania.

		// Zapis do pliku.
		save_measures_to_file();
		return false;
	};//: if
	// Oczekiwanie na odczyt aktualnego polozenia koncowki.
	if ( idle_step_counter )
	{
		// Przygotowanie nastepnego polecenia.

		idle_step_counter--;
		return true;
	};//: if
	// Pobranie odczytow.
 
	// Przepisanie odczytow do wektora.
	two_robots_measure current_measure;
	memcpy(current_measure.irp6ot, irp6ot->ecp_td.current_XYZ_ZYZ_arm_coordinates, 6*sizeof(double));
	memcpy(current_measure.irp6p, irp6p->ecp_td.current_XYZ_ZYZ_arm_coordinates, 6*sizeof(double));
/*	std::cout<<"current_measure"<<std::endl;
	for(int i=0; i<6; i++)
		std::cout<< current_measure.irp6ot[i] <<"\t";
	for(int i=0; i<6; i++)
		std::cout<< current_measure.irp6p[i] <<"\t";
	std::cout<<std::endl;*/
	// Roznica - na kazdej wspolrzednej na razie taka sama.
	double eps = 5e-3;
	bool add_measurement = false;
	// Porownanie obecnych polozen z poprzednimi.
	for(int i=0; i<6; i++)
		if (fabs(last_measure.irp6ot[i] - current_measure.irp6ot[i]) > eps)
		{
			add_measurement = true;
			break;
		}//: if
	// Ewenualne dodanie bierzacych pomiarow.
	if (add_measurement)
	{
		measures.push_back(current_measure);
		// Przepisanie obecnych pomiarow na poprzednie.
		memcpy(&last_measure, &current_measure, sizeof(two_robots_measure));
		// Informacja dla uzytkownika - beep.
		std::cout<<"\a"<<std::endl;
	}
	// Przygotowanie nastepnego polecenia.

	// Nastepny krok.
	return true;
}; //: next_step


// Zapis pomiarow do pliku.
void mp_two_robots_measures_generator::save_measures_to_file (void)
{
	// Przesylka z ECP do UI
	ECP_message ecp_to_ui_msg;
	// Odpowiedz UI do ECP
	UI_reply ui_to_ecp_rep;
	ecp_to_ui_msg.hdr.type=0;
	// Polecenie wprowadzenia nazwy pliku
	ecp_to_ui_msg.ecp_message = SAVE_FILE;
	// Wzorzec nazwy pliku
	strcpy(ecp_to_ui_msg.string,"*.*");
	// Wyslanie polecenia pokazania okna wyboru pliku.
	if (MsgSend(UI_fd, &ecp_to_ui_msg,  sizeof(ECP_message),  &ui_to_ecp_rep, sizeof(UI_reply)) < 0)
	{
		sr_ecp_msg.message (SYSTEM_ERROR, errno, "Send to UI failed");
		throw mp_generator::MP_error(SYSTEM_ERROR, (uint64_t) 0);
	}//: if
	// Sprawdzenie katalogu.
	if ( chdir(ui_to_ecp_rep.path) != 0 )
	{
		sr_ecp_msg.message (NON_FATAL_ERROR, NON_EXISTENT_DIRECTORY);
		return;
	};//: if
	// Otworzenie plik do zapisu.
	ofstream to_file(ui_to_ecp_rep.filename); 
	if (to_file == NULL)
	{
		sr_ecp_msg.message (NON_FATAL_ERROR, NON_EXISTENT_FILE);
		return;
	};//: if
	for(int m=0; m<measures.size(); m++)
	{
		// Pobranie danego pomiaru.
		two_robots_measure trm = (two_robots_measure)measures[m];
		for(int i=0; i<6; i++)
			to_file<< trm.irp6ot[i] <<"\t";
		for(int i=0; i<6; i++)
			to_file<< trm.irp6p[i] <<"\t";
		to_file<<"\n";
	};//: for
	to_file.close();
	sr_ecp_msg.message("Measures were saved to file");
};//: save_measures_to_file
