/* ! \file include/edp/ati6284/edp_s.h
 * \brief plik naglwkowy zawierajacy definicje klasy dla czujnika ati6284
 * Ostatnia modyfikacja: 04.2006 */

#if !defined(_EDP_S_ATI6284_H)
#define _EDP_S_ATI6284_H

#include "edp/common/edp_force_sensor.h"

namespace mrrocpp {
namespace edp {
namespace sensor {

class ATI6284_force : public force {

private:
	unsigned uCount;      //!<  zmienna indeksuj�ca
	unsigned uStatus;     //!< wskazanie zape�nienia kolejki
	float last_correct[6];
	short int overload;
	short int show_no_result;

	unsigned Samples_Acquired, Total_Number_of_Samples;
	short int uValues[6]; //!<  binarne napi�cie otrzymane z karty
	float sVolt[6];        //!<  tablica wartosi podanych w Voltach
	float sBias[6];        //!<  tablica Bias

	int irq_no;
	int szafa_id;
	unsigned short int index;

	//!<  Deklaracje funkcji s�u��cych do komunikacji i ustawiania karty akwizycji danych
	void Configure_Board(void);        //!< ustawienie karty
	void MSC_Clock_Configure(void);    //!< konfiguracja zegar�w karty
	void AI_Reset_All(void);           //!< wykasowanie wszystkich rejestr�w
	void AI_Board_Personalize(void);   //!< ustawienie karty
	void AI_Initialize_Configuration_Memory_Output(void);  //!< inicjalizacaja rejestr�w
	void AI_Board_Environmentalize(void);                  //!< ustawienie parametr�w karty
	void AI_Trigger_Signals(void);     //!< sygna�y wyzwalaj�ce
	void Number_of_Scans(void);        //!< ustawienie ilo�ci skan�w
	void AI_Scan_Start(void);          //!< start skanowania
	void AI_End_of_Scan(void);         //!< koniec skanowania
	void Convert_Signal(void);         //!< konwersja sygna�u
	void Clear_FIFO(void);             //!< wyczyszczenie kolejki FIFO
	void AI_Interrupt_Enable(void);    //!< uruchomienie przerwa�
	void AI_Arming(void);              //!< uruchomienie wej�� analogowych karty
	void Interrupt_Service_Routine(void);  //!<  Interrupt service routine
	void AI_Start_The_Acquisition(void);  //!< start akwizycji
	void InitMite(void);               //!<  Inicjalizacja chipu Mite
	void Input_to_Volts(void);         //!< przekszta�cenie wej�ciowych danych na wolty

public:
	void connect_to_hardware (void);

	ATI6284_force(common::manip_effector &_master);       //!< konstruktor uruchamiaj�cy czujnik i ustawiaj�cuy bias
	virtual ~ATI6284_force();    //!< destruktor od��czaj�cy kart� z magistrali PCI

	void configure_sensor (void);           //!<  konfiguracja czujnika
	void wait_for_event(void);              //!<  oczekiwanie na zdarzenie
	void initiate_reading (void);           //!<  zadanie odczytu od VSP
	void get_reading (void);                //!<  odebranie odczytu od VSP
}; //!<  end: class vsp_sensor

} // namespace sensor
} // namespace edp
} // namespace mrrocpp

#endif
