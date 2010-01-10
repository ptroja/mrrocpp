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

class servo_buffer: public common::servo_buffer
{
	// Bufor polecen przysylanych z EDP_MASTER dla SERVO
	// Obiekt z algorytmem regulacji

public:
	// output_buffer
	effector &master;
	void load_hardware_interface(void);
	servo_buffer(effector &_master); // konstruktor


	void synchronise(void); // synchronizacja

	// obliczenie nastepnej wartosci zadanej dla wszystkich napedow
};

} // namespace common
} // namespace edp
} // namespace mrrocpp


#endif
