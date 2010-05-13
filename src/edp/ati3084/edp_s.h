// -------------------------------------------------------------------------
//
//
// Definicje klasy edp_ATI3084_force_sensor
//
// Ostatnia modyfikacja: 2005
// Autor: yoyek
// -------------------------------------------------------------------------

#if !defined(_EDP_S_ATI3084_H)
#define _EDP_S_ATI3084_H

#include "edp/common/edp_irp6s_postument_track.h"

namespace mrrocpp {
namespace edp {
namespace sensor {

// server answers
#define COMMAND_OK 0x00

// error types
#define __ERROR_COM	1
#define __ERROR_SEND	2
#define __ERROR_NO_ANSWER	3
#define __ERROR_MEASURE	4
#define __ERROR_BAD_COMMAND	5
#define __ERROR_INIT_SEND 6
#define __ERROR_INIT_COM	7
#define __ERROR_BUFFER_MAX	8
#define __ERROR_BUFFER_ZERO 9

#define SERIAL 1 // PARALLEL - wysylynie po zlaczu rownoleglym - dziala slabo,
// SERIAL - wysylanie po zlaczu szeregowym - dziala dobrze

#define SCHUNK_INTR_TIMEOUT_HIGH 10000000
#define SCHUNK_INTR_TIMEOUT_LOW  10000000

// Z PLIKU cz_lib.cc

#define CL_0 "CL 0\r"
#define CF_1 "CF 1\r"
#define CD_B "CD B\r"
#define CD_R "CD R\r"
#define CV_6 "CV 3F\r"
#define SA "SA 0\r"
#define SM "SM 1\r"
#define CP_P "CP P\r"
#define SB	"SB\r"
#define SU	"SU\r"
#define SZ	"SZ\r"
#define CR	"\r"

#define RESET	"\027"
#define SGET1	"\024"
#define GET1	"QR\r"
#define GETN	"QS\r"
#define YESCOMM	"Y\r"

// by Y dla karty advantech

#define COM_NR 1

#define LOWER_OUTPUT 0
#define LOWER_INPUT 1
#define UPPER_OUTPUT 4
#define UPPER_INPUT 5
#define CONTROL_OUTPUT 2
#define STB_PORT_INPUT 6
#define ACK_PORT_INPUT 2

#define PORT_0_CONFIG 3
#define PORT_1_CONFIG 7
#define INTER_CONFIG 32

#define INTR_LOOP_DELAY 1000 // opoznienie pomiedzy dwoma stanami wyjscia karty w funkcji obslugi przerwania
#define INTR_NS_DELAY 10000 // opoznienie pomiedzy dwoma stanami wyjscia karty
// w funkcji obslugi przerwania w nanosekundach


/********** klasa czujnikow po stronie VSP **************/
class ATI3084_force : public force {

private:
	unsigned int int_attached;// informacja o tym, czy obsluga przerwanie jest juz przypisana

	int pidx; // do obslugi karty advantech pci1751
	void* hdl; // wlasciwy uchwyt do danego urzadzenia
	int phdl; // pci handle -> fd do servera PCI

	void set_output(uint16_t value);
	void set_obf(unsigned char state);
	bool check_ack(void);
	void initiate_registers(void);
	void solve_transducer_controller_failure(void);
	void check_cs(void);
	void parallel_do_send_command(const char* command);

	short do_Wait(const char* command);// by old schunk
	short do_send_command(const char* command);
	short do_init(void);

	//! serial port file descriptor
	int uart;

	short ERROR_CODE;

	struct sigevent tim_event;

public:

	void connect_to_hardware (void);

	ATI3084_force(common::manip_effector &_master);
	virtual ~ATI3084_force();

	void configure_sensor (void);	// konfiguracja czujnika
	void wait_for_event(void);		// oczekiwanie na zdarzenie
	void initiate_reading (void);		// zadanie odczytu od VSP
	void get_reading (void);			// odebranie odczytu od VSP		// zwraca blad

}; // end: class vsp_sensor

// na zewnatrz klasy gdyz odwoluje sie do nich funkcja obslugi przerwania
bool check_intr(void);
bool check_stb(void);
void clear_intr(void);
void set_ibf(unsigned char state);
short get_input(void);

} // namespace sensor
} // namespace edp
} // namespace mrrocpp


#endif
