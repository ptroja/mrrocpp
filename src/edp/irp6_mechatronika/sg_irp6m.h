// -------------------------------------------------------------------------
//                            sg_local.h
// Definicje struktur danych i metod dla procesu EDP mechatronika
//
// Ostatnia modyfikacja: 2006
// -------------------------------------------------------------------------



#ifndef __SG_IRP6M_H
#define __SG_IRP6M_H

#include "edp/common/edp.h"

#include "edp/common/servo_gr.h"

namespace mrrocpp {
namespace edp {
namespace irp6m {
class effector;
// numeracja od 2 ze wzgledu na analogie irp6_postument


#define IRP6_MECHATRONIKA_AXIS_0_TO_5_INC_PER_REVOLUTION  2000.0  // Liczba impulsow enkodera na obrot walu - musi byc float

// os od ktorej startuje synchronizacja - numeracja od 0
#define IRP6M_SYN_INIT_AXE 1

// numeracja od 2 ze wzgledu na analogie irp6_postument


class servo_buffer: public common::servo_buffer
{
		// Bufor polecen przysylanych z EDP_MASTER dla SERVO
		// Obiekt z algorytmem regulacji

		uint8_t Move_a_step(void); // wykonac ruch o krok nie reagujac na SYNCHRO_SWITCH i SYNCHRO_T

	public:
		// output_buffer
		void get_all_positions(void);
		effector &master;
	    void load_hardware_interface (void);
		servo_buffer(effector &_master); // konstruktor
		~servo_buffer(void); // destruktor

		void synchronise(void); // synchronizacja
		uint64_t compute_all_set_values(void);
		// obliczenie nastepnej wartosci zadanej dla wszystkich napedow
};

} // namespace common
} // namespace edp
} // namespace mrrocpp



#endif
