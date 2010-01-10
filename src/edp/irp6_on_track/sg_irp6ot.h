// -------------------------------------------------------------------------
//                            sg_local.h
// Definicje struktur danych i metod dla procesu EDP on_track
//
// Ostatnia modyfikacja: 2006
// -------------------------------------------------------------------------



#ifndef __SG_IRP6OT_H
#define __SG_IRP6OT_H

#include "edp/common/edp.h"
#include "edp/common/servo_gr.h"


namespace mrrocpp {
namespace edp {
namespace irp6ot {
class effector;



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


		void synchronise(void); // synchronizacja
		uint64_t compute_all_set_values(void);
		// obliczenie nastepnej wartosci zadanej dla wszystkich napedow
};

} // namespace irp6ot
} // namespace edp
} // namespace mrrocpp



#endif
