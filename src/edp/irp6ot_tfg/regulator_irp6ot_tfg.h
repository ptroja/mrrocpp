// -------------------------------------------------------------------------
//                            sg_local.h
// Definicje struktur danych i metod dla procesu EDP on_track
//
// Ostatnia modyfikacja: 2006
// -------------------------------------------------------------------------



#ifndef __REGULATOR_IRP6OT_TFG_H
#define __REGULATOR_IRP6OT_TFG_H
#include "edp/common/edp.h"
#include "edp/common/regulator.h"


namespace mrrocpp {
namespace edp {
namespace irp6ot_tfg {
class effector;


#define IRP6_ON_TRACK_AXE8_MAX_PWM_INCREMENT	1000

#define IRP6_ON_TRACK_AXIS_7_SYNCHRO_STEP_COARSE -0.5
#define IRP6_ON_TRACK_AXIS_7_SYNCHRO_STEP_FINE   -0.2

// stale dla automatu w regulatorze chwytka
#define GRIPPER_BLOCKED_TIME_PERIOD 200
#define MAX_GRIPPER_SUM_OF_CURRENTS_NR_OF_ELEMENTS 1000

#define IRP6_ON_TRACK_GRIPPER_SUM_OF_CURRENTS_NR_OF_ELEMENTS 10
#define IRP6_ON_TRACK_GRIPPER_SUM_OF_CURRENTS_MAX_VALUE 100
// #define IRP6_ON_TRACK_GRIPPER_SUM_PWM_MIN 80


#


#define POSTUMENT_TO_TRACK_VOLTAGE_RATIO 0.75


/*-----------------------------------------------------------------------*/
class NL_regulator_8_irp6ot: public common::NL_regulator
{
    /* Klasa regulatorow konkretnych */
    // Obiekt z algorytmem regulacji

    long gripper_blocked_start_time;
    int sum_of_currents, current_index;
    int currents [MAX_GRIPPER_SUM_OF_CURRENTS_NR_OF_ELEMENTS];


public:
    NL_regulator_8_irp6ot (uint8_t reg_no, uint8_t reg_par_no,
                           double aa, double bb0, double bb1, double k_ff, common::motor_driven_effector &_master); // konstruktor

    virtual uint8_t compute_set_value ( void );
    // obliczenie nastepnej wartosci zadanej dla napedu - metoda konkretna

}
; // end: class NL_regulator_8
// ----------------------------------------------------------------------


} // namespace irp6ot
} // namespace edp
} // namespace mrrocpp



#endif
