// -------------------------------------------------------------------------
//                            hi_rydz.h
// Definicje struktur danych i metod dla interfejsu sprzetowego dla robota conveyor
//
// Ostatnia modyfikacja: 2005
// -------------------------------------------------------------------------

#ifndef __HI_LOCAL_CONV_H
#define __HI_LOCAL_CONV_H

#include "edp/common/hi_rydz.h"

namespace mrrocpp {
namespace edp {
namespace conveyor {

// Struktury danych wykorzystywane w hardware_interface
const int IRQ_REAL = 10; // Numer przerwania sprzetowego
const unsigned short int INT_FREC_DIVIDER = 8; // mnoznik czestotliwosci przerwan (odpowiada 2ms)

#define HI_RYDZ_INTR_TIMEOUT_HIGH 10000000 // by Y - timeout przerwania z szafy badz zegara

#define FIRST_SERVO_PTR           0xC0
#define INTERRUPT_GENERATOR_SERVO_PTR	 0xC0


#define ISA_CARD_OFFSET 0x20 // w zaleznosci od ustawienia na karcie isa
#define ADR_OF_SERVO_PTR          0x305 + ISA_CARD_OFFSET
#define SERVO_COMMAND1_ADR       0x200 + ISA_CARD_OFFSET
#define SERVO_COMMAND2_ADR       0x202 + ISA_CARD_OFFSET
#define SERVO_REPLY_STATUS_ADR    0x200 + ISA_CARD_OFFSET
#define SERVO_REPLY_INT_ADR       0x202 + ISA_CARD_OFFSET
#define SERVO_REPLY_POS_LOW_ADR   0x204 + ISA_CARD_OFFSET
#define SERVO_REPLY_POS_HIGH_ADR  0x206 + ISA_CARD_OFFSET
#define SERVO_REPLY_REG_1_ADR     0x208 + ISA_CARD_OFFSET
#define SERVO_REPLY_REG_2_ADR     0x20A + ISA_CARD_OFFSET

// Polecenia dla sterownikow mikroprocesorowych osi
#define RESET_POSITION_COUNTER    0x0400 // Zerowanie licznika polozenia
#define RESET_MANUAL_MODE         0x0800 // Zerowanie pracy recznej
#define RESET_ALARM               0x0C00 // Zerowanie alarmu sytuacji awaryjnej
#define PROHIBIT_MANUAL_MODE      0x1000 // Zakaz pracy recznej
#define ALLOW_MANUAL_MODE         0x1400 // Zezwolenie na prace reczna
#define START_SYNCHRO             0x1800 // Rozpoczecie synchronizacji
#define FINISH_SYNCHRO            0x1C00 // Zakoncz synchronizacje osi
#define SET_INT_FREQUENCY         0x2000 // Ustaw dzielnik czestotliowsci przerwan
#define SET_MAX_CURRENT           0x2400 // Ustaw prad maksymalny
#define START_CLOCK_INTERRUPTS           0x5000 // Wlacz przerwania zegarowe
#define STOP_CLOCK_INTERRUPTS           0x5400 // Wylacz przerwania zegarowe

#define LM629_VIA_MICROCONTROLLER_MODE 0x4C01
#define MICROCONTROLLER_MODE 0x4C00
#define ZERO_ORDER 0x0000

#define STOP_MOTORS               0x0200 // Zatrzymanie silnikow (W.S. ???)

#define CONVEYOR_AXIS_1_MAX_CURRENT 0x2460
// 13,7 j na A

// ------------------------------------------------------------------------
//                HARDWARE_INTERFACE class
// ------------------------------------------------------------------------

class hardware_interface: public common::hardware_interface {

public:
	hardware_interface(effector &_master); // Konstruktor
	~hardware_interface(void); // Destruktor
	bool is_hardware_error(void); // Sprawdzenie czy wystapil blad sprzetowy

	uint64_t read_write_hardware(void); // Obsluga sprzetu
	void reset_counters(void); // Zerowanie licznikow polozenia


	// oczekiwanie na przerwanie - tryb obslugi i delay(lag) po odebraniu przerwania
	int hi_int_wait(int inter_mode, int lag);

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


} // namespace conveyor
} // namespace edp
} // namespace mrrocpp

#endif // __HI_RYDZ_H
