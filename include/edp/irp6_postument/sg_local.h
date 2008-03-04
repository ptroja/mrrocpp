// -------------------------------------------------------------------------
//                            sg_local.h
// Definicje struktur danych i metod dla procesu EDP postument
//
// Ostatnia modyfikacja: 2006
// -------------------------------------------------------------------------



#ifndef __SG_IRP6P_H
#define __SG_IRP6P_H

#include "edp/common/edp.h"
#include "edp/common/sg_irp6p_and_conv.h"

// os od ktorej startuje synchronizacja - numeracja od 0
#define IRP6P_SYN_INIT_AXE 1


/************************ EDP_SPEAKER ****************************/
class irp6p_servo_buffer  : public servo_buffer
{
    // Bufor polecen przysylanych z EDP_MASTER dla SERVO
    // Obiekt z algorytmem regulacji


    BYTE Move_a_step (void);         // wykonac ruch o krok nie reagujac na SYNCHRO_SWITCH i SYNCHRO_T

public:

    // output_buffer
    void get_all_positions (void);
    edp_irp6p_effector &master;

    irp6p_servo_buffer (edp_irp6p_effector &_master);             // konstruktor
    ~irp6p_servo_buffer (void);      // destruktor

    void synchronise (void);         // synchronizacja
    uint64_t compute_all_set_values (void);
    // obliczenie nastepnej wartosci zadanej dla wszystkich napedow


}
; // end: class servo_buffer
/************************ EDP_SPEAKER ****************************/

#endif
