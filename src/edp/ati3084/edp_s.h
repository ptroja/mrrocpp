// -------------------------------------------------------------------------
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

// Z PLIKU cz_defs.h

// user commands
// #define INIT	0x01
#define COMGETS1 0x01
#define COMGET1	0x02
#define COMGETN	0x03
#define COMGIVEN	0x04
#define COMBIAS	0x05
#define COMUNBIAS 0x06
#define COMDELBIAS 0x07
#define COMAVERAGE 0x08
#define COMRESET 0x09

#define COMSERVER_END 0xFD

// server answers
#define COMMAND_OK 0x00
#define COMMAND_ERROR	0xFF

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


// max & min svar values
#define MAX_SVAR_VALUE 0x7F // +127
#define MIN_SVAR_VALUE 0x80 // -128

// Z PLIKU cz_lib.h

// PARALLEL - wysylynie po zlaczu rownoleglym - dziala slabo,
// SERIAL - wysylanie po zlaczu szeregowym - dziala dobrze
#define SERIAL 1

#define NUM_SVAR	6
#define MAX_NUM_MEASURE 1000

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

#define PICMASK 0x21
#define PICEOI	0x20
#define EOI	0x20

#define TIMEOUT 500
#define MAX_BUFFER	3

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

// opoznienie pomiedzy dwoma stanami wyjscia karty
// w funkcji obslugi przerwania w nanosekundach
#define INTR_NS_DELAY 10000

#define MDS_DATA_RANGE 20

/********** klasa czujnikow po stronie VSP **************/
class ATI3084_force : public force {

public:

	//! data strucutre shared between thread and interrupt handler
	typedef struct mds_data
	{
		int intr_mode;
		int byte_counter;
		int is_received;
		short data[MDS_DATA_RANGE];

		//! spinlock to for disabling interrupts
		intrspin_t spinlock;
	} mds_data_t;

	void connect_to_hardware (void);

	ATI3084_force(common::manip_effector &_master);
	virtual ~ATI3084_force();

	void configure_sensor (void);	// konfiguracja czujnika
	void wait_for_event(void);		// oczekiwanie na zdarzenie
	void initiate_reading (void);		// zadanie odczytu od VSP
	void get_reading (void);			// odebranie odczytu od VSP		// zwraca blad

private:
	//! data shared between thread and interrupt handler
	mds_data_t mds;

	//! interrupt id
	int interrupt_id;

	//! interrupt timeout
	uint64_t int_timeout;

	//! informacja o tym, czy obsluga przerwanie jest juz przypisana
	bool int_attached;

	// Z PLIKU cz_lib.h
	int LSREG;
	int LCREG;	 	/* Line Control Register */
	int IEREG;	 	/* Interrupt Enable Register */
	int MCREG;	 	/* Modem Control Register */
	int FCREG;	 	/* FIFO Control Register */
	int TxBUF;	 	/* Transmit Buffer */
	int RxBUF;	 	/* Receive Buffer */
	int DIVLSB;	 	/* Divisor Least Sign. Byte */
	int DIVMSB; 	/* Divisor Most Sign. Byte */
	int INT_NUM;
	int NOT_IRQ;
	// KONIEC Z PLIKU

	//! do obslugi karty advantech pci1751
	int pidx;

	//! wlasciwy uchwyt do danego urzadzenia
	void* hdl;

	//! // pci handle -> fd do servera PCI
	int phdl;

	void set_char_output(char* znak);
	void set_output(short value);
	void set_obf(unsigned char state);
	bool check_ack(void);
	void initiate_registers(void);
	void solve_transducer_controller_failure(void);
	void check_cs(void);
	void parallel_do_send_command(const char* command);

	short do_Wait(const char* command);// by old schunk
	short do_send_command(const char* command);
	short do_init(void);

	short ERROR_CODE;

	struct sigevent tim_event;
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
