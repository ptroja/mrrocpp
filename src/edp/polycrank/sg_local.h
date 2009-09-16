// -------------------------------------------------------------------------
//                            sg_local.h
// Definicje struktur danych i metod dla procesu EDP polycrank
//
// Ostatnia modyfikacja: 2006
// -------------------------------------------------------------------------



#ifndef __SG_IRP6M_H
#define __SG_IRP6M_H

#include "edp/common/edp.h"

#include "edp/common/servo_gr.h"

namespace mrrocpp {
namespace edp {
namespace polycrank {

// numeracja od 2 ze wzgledu na analogie irp6_postument

// ograniczenia przyrostu PWM dla POLYCRANK
#define POLYCRANK_AXIS2_MAX_PWM_INCREMENT	1000
#define POLYCRANK_AXIS3_MAX_PWM_INCREMENT	1000
#define POLYCRANK_AXIS4_MAX_PWM_INCREMENT	1000
#define POLYCRANK_AXIS5_MAX_PWM_INCREMENT	1000
#define POLYCRANK_AXIS6_MAX_PWM_INCREMENT	1000

// Stale dla celow synchronizacji POLYCRANK
#define POLYCRANK_SYNCHRO_STEP_COARSE -0.03
#define POLYCRANK_SYNCHRO_STEP_FINE   -0.007

// os od ktorej startuje synchronizacja - numeracja od 0
#define IRP6M_SYN_INIT_AXIS 1

// numeracja od 2 ze wzgledu na analogie irp6_postument

/*-----------------------------------------------------------------------*/
class NL_regulator_2_irp6m: public common::NL_regulator
{
    /* Klasa regulatorow konkretnych */
    // Obiekt z algorytmem regulacji

public:
    NL_regulator_2_irp6m (lib::BYTE reg_no, lib::BYTE reg_par_no,
                          double aa, double bb0, double bb1, double k_ff, common::manip_and_conv_effector &_master); // konstruktor

    virtual lib::BYTE compute_set_value ( void );
    // obliczenie nastepnej wartosci zadanej dla napedu - metoda konkretna

};
// ----------------------------------------------------------------------

/*-----------------------------------------------------------------------*/
class NL_regulator_3_irp6m: public common::NL_regulator
{
    /* Klasa regulatorow konkretnych */
    // Obiekt z algorytmem regulacji

public:
    NL_regulator_3_irp6m (lib::BYTE reg_no, lib::BYTE reg_par_no,
                          double aa, double bb0, double bb1, double k_ff, common::manip_and_conv_effector &_master); // konstruktor

    virtual lib::BYTE compute_set_value ( void );
    // obliczenie nastepnej wartosci zadanej dla napedu - metoda konkretna

};
// ----------------------------------------------------------------------

/*-----------------------------------------------------------------------*/
class NL_regulator_4_irp6m: public common::NL_regulator
{
    /* Klasa regulatorow konkretnych */
    // Obiekt z algorytmem regulacji

public:
    NL_regulator_4_irp6m (lib::BYTE reg_no, lib::BYTE reg_par_no,
                          double aa, double bb0, double bb1, double k_ff, common::manip_and_conv_effector &_master); // konstruktor

    virtual lib::BYTE compute_set_value ( void );
    // obliczenie nastepnej wartosci zadanej dla napedu - metoda konkretna

};
// ----------------------------------------------------------------------


/*-----------------------------------------------------------------------*/
class NL_regulator_5_irp6m: public common::NL_regulator
{
    /* Klasa regulatorow konkretnych */
    // Obiekt z algorytmem regulacji

public:
    bool first;
    NL_regulator_5_irp6m (lib::BYTE reg_no, lib::BYTE reg_par_no,
                          double aa, double bb0, double bb1, double k_ff, common::manip_and_conv_effector &_master); // konstruktor

    virtual lib::BYTE compute_set_value ( void );
    // obliczenie nastepnej wartosci zadanej dla napedu - metoda konkretna

};
// ----------------------------------------------------------------------


/*-----------------------------------------------------------------------*/
class NL_regulator_6_irp6m: public common::NL_regulator
{
    /* Klasa regulatorow konkretnych */
    // Obiekt z algorytmem regulacji

public:
    NL_regulator_6_irp6m (lib::BYTE reg_no, lib::BYTE reg_par_no,
                          double aa, double bb0, double bb1, double k_ff, common::manip_and_conv_effector &_master); // konstruktor

    virtual lib::BYTE compute_set_value ( void );
    // obliczenie nastepnej wartosci zadanej dla napedu - metoda konkretna

};
// ----------------------------------------------------------------------




class servo_buffer : public common::servo_buffer
{
		// Bufor polecen przysylanych z EDP_MASTER dla SERVO
		// Obiekt z algorytmem regulacji

		lib::BYTE Move_a_step(void); // wykonac ruch o krok nie reagujac na SYNCHRO_SWITCH i SYNCHRO_T

	public:
		effector &master;
		// output_buffer
		void get_all_positions(void);

		servo_buffer(effector &_master); // konstruktor
		~servo_buffer(void); // destruktor

		void synchronise(void); // synchronizacja
		uint64_t compute_all_set_values(void);
		// obliczenie nastepnej wartosci zadanej dla wszystkich napedow
};

} // namespace polycrank
} // namespace edp
} // namespace mrrocpp



#endif
