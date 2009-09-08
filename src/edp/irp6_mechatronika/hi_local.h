// -------------------------------------------------------------------------
//                            hi_rydz.h
// Definicje struktur danych i metod dla interfejsu sprzetowego dla robota irp6 mechatronika
//
// Ostatnia modyfikacja: 16.04.98
// -------------------------------------------------------------------------

#ifndef __HI_LOCAL_IRP6M_H
#define __HI_LOCAL_IRP6M_H

#include <signal.h>
#include <time.h>

#include "edp/common/hi_rydz.h"

namespace mrrocpp {
namespace edp {
namespace irp6m {

// Struktury danych wykorzystywane w hardware_interface
const int IRQ_REAL = 10; // Numer przerwania sprzetowego
const unsigned short int INT_FREC_DIVIDER = 8; // mnoznik czestotliwosci przerwan (odpowiada 2ms)

#define HI_RYDZ_INTR_TIMEOUT_HIGH 10000000 // by Y - timeout przerwania z szafy badz zegara

#define FIRST_SERVO_PTR           0xC1
#define INTERRUPT_GENERATOR_SERVO_PTR	 0xC1


#define ISA_CARD_OFFSET 0x10 // w zaleznosci od ustawienia na karcie isa

#define IRP6_MECHATRONIKA_AXIS_1_MAX_CURRENT           0x2430 // ustawienie pradu maksymalnego dla przedostatniej osi - obrot chwytaka
#define IRP6_MECHATRONIKA_AXIS_2_MAX_CURRENT           0x2430 // ustawienie pradu maksymalnego dla przedostatniej osi - obrot chwytaka
#define IRP6_MECHATRONIKA_AXIS_3_MAX_CURRENT           0x2430 // ustawienie pradu maksymalnego dla przedostatniej osi - obrot chwytaka
#define IRP6_MECHATRONIKA_AXIS_4_MAX_CURRENT           0x2430 // ustawienie pradu maksymalnego dla przedostatniej osi - obrot chwytaka
#define IRP6_MECHATRONIKA_AXIS_5_MAX_CURRENT           0x2430 // ustawienie pradu maksymalnego dla przedostatniej osi - obrot chwytaka
#define IRP6_MECHATRONIKA_AXIS_6_MAX_CURRENT           0x2410 // ustawienie pradu maksymalnego dla przedostatniej osi - obrot chwytaka
// 13,7 j na amper

#define IRP6_MECHATRONIKA_AXIS_7_MAX_CURRENT           0x2420 // ustawienie pradu maksymalnego dla zacisku chwytaka
// 34,7 j na 100ma, streafa nieczulosci 40ma

// ------------------------------------------------------------------------
//                HARDWARE_INTERFACE class
// ------------------------------------------------------------------------

#define SIG	SIGRTMIN

class hardware_interface: public common::hardware_interface {
#ifndef __QNXNTO__
	//! periodic timer
	timer_t timerid;
	sigset_t mask;
#endif
public:
	hardware_interface(effector &_master); // Konstruktor
	~hardware_interface(void); // Destruktor
	bool is_hardware_error(void); // Sprawdzenie czy wystapil blad sprzetowy

	uint64_t read_write_hardware(void); // Obsluga sprzetu
	void reset_counters(void); // Zerowanie licznikow polozenia

	void start_synchro(int drive_number);

	// synchronizacja automatyczna z wykrorzystaniem lm629
	int synchronise_via_lm629(void);

	// oczekiwanie na przerwanie - tryb obslugi i delay(lag) po odebraniu przerwania
	int hi_int_wait(int inter_mode, int lag);

	void finish_synchro(int drive_number);

private:
	edp::common::irq_data_t irq_data;

}; // koniec: class hardware_interface

#ifdef __cplusplus
extern "C"
{
#endif
    // pid_t far int_handler (void);  // Obsluga przerwania
    // by YOYEK & 7 - zastapic inna procedura obslugi prrzerwania

    const struct sigevent *
                int_handler (void *arg, int id); // by YOYEK & 7 - nowa forma z helpu

#ifdef __cplusplus
}
#endif


} // namespace irp6m
} // namespace edp
} // namespace mrrocpp

#endif // __HI_RYDZ_H
