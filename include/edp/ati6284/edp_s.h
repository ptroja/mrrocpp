/* ! \file include/edp/ati6284/edp_s.h
    * \brief plik nag³ówkowy zawieraj¹cy definicjê klasy dla czujnika ati6284
    * Ostatnia modyfikacja: 04.2006 */

#if !defined(_EDP_S_ATI6284_H)
#define _EDP_S_ATI6284_H

#include <sys/siginfo.h>

#include "edp/common/edp_force_sensor.h"

class edp_ATI6284_force_sensor : public edp_force_sensor{

private:
	unsigned  uCount;      //!<  zmienna indeksuj¹ca
	unsigned  uStatus;     //!< wskazanie zape³nienia kolejki
	float last_correct[6];
	short int overload ;
	short int show_no_result;

	unsigned Samples_Acquired, Total_Number_of_Samples;
	short int  uValues[6]; //!<  binarne napiêcie otrzymane z karty
	float sVolt[6];        //!<  tablica wartosi podanych w Voltach 
	float sBias[6];        //!<  tablica Bias
		
public:
	short int id;
	short int irq_no;
	short int szafa_id;
	short int index;
	char *calfilepath;     //!< œcie¿ka do pliku zawieraj¹cego parametry konfiguracyjne czujnika

	short ERROR_CODE;
	
	edp_ATI6284_force_sensor(edp_irp6s_postument_track_effector &_master);       //!< konstruktor uruchamiaj¹cy czujnik i ustawiaj¹cuy bias
	virtual ~edp_ATI6284_force_sensor();    //!< destruktor od³¹czaj¹cy kartê z magistrali PCI

	void configure_sensor (void);           //!<  konfiguracja czujnika
	void wait_for_event(void);              //!<  oczekiwanie na zdarzenie
	void initiate_reading (void);           //!<  zadanie odczytu od VSP
	void get_reading (void);                //!<  odebranie odczytu od VSP 
	void terminate (void);                  //!<  rozkaz zakonczenia procesu VSP

	//!<  Deklaracje funkcji s³u¿¹cych do komunikacji i ustawiania karty akwizycji danych
	void Configure_Board(void);        //!< ustawienie karty
	void MSC_Clock_Configure(void);    //!< konfiguracja zegarów karty
	void AI_Reset_All(void);           //!< wykasowanie wszystkich rejestrów
	void AI_Board_Personalize(void);   //!< ustawienie karty
	void AI_Initialize_Configuration_Memory_Output(void);  //!< inicjalizacaja rejestrów
	void AI_Board_Environmentalize(void);                  //!< ustawienie parametrów karty
	void AI_Trigger_Signals(void);     //!< sygna³y wyzwalaj¹ce
	void Number_of_Scans(void);        //!< ustawienie iloœci skanów
	void AI_Scan_Start(void);          //!< start skanowania
	void AI_End_of_Scan(void);         //!< koniec skanowania
	void Convert_Signal(void);         //!< konwersja sygna³u
	void Clear_FIFO(void);             //!< wyczyszczenie kolejki FIFO
	void AI_Interrupt_Enable(void);    //!< uruchomienie przerwañ
	void AI_Arming(void);              //!< uruchomienie wejœæ analogowych karty
	void Interrupt_Service_Routine(void);                  //!<  Interrupt service routine
	void AI_Start_The_Acquisition(void);                   //!< start akwizycji
	void InitMite(void);                                   //!<  Inicjalizacja chipu Mite
	void Input_to_Volts(void);         //!< przekszta³cenie wejœciowych danych na wolty

}; //!<  end: class vsp_sensor

	const struct sigevent *isr_handler (void *area, int id);         //!< przerwanie od karty
	const struct sigevent *szafa_handler (void *area, int szafa_id); //!< przerwanie od szafy
	
#endif
