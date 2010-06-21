// -------------------------------------------------------------------------
//                            sg_local.h
// Definicje struktur danych i metod dla procesu EDP on_track
//
// Ostatnia modyfikacja: 2006
// -------------------------------------------------------------------------


#ifndef __SG_IRP6OT_TFG_H
#define __SG_IRP6OT_TFG_H

#include "edp/common/edp.h"
#include "edp/common/servo_gr.h"

namespace mrrocpp {
namespace edp {
namespace irp6ot_tfg {
class effector;

class servo_buffer: public common::servo_buffer
{
	// Bufor polecen przysylanych z EDP_MASTER dla SERVO
	// Obiekt z algorytmem regulacji

public:
	// output_buffer
	void get_all_positions(void);
	effector &master;
	void load_hardware_interface(void);
	servo_buffer(effector &_master); // konstruktor
	// obliczenie nastepnej wartosci zadanej dla wszystkich napedow
};

} // namespace irp6ot
} // namespace edp
} // namespace mrrocpp


#endif
