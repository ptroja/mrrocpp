// -------------------------------------------------------------------------
//                            servo_gr.h
// Definicje struktur danych i metod dla procesu SERVO_GROUP
//
// Ostatnia modyfikacja: 16.04.98
// -------------------------------------------------------------------------

#ifndef __EDP_REGULATOR_H
#define __EDP_REGULATOR_H

#include "edp/common/edp.h"
#include "edp/common/hi_rydz.h"
#include "edp/common/edp_extension_thread.h"

namespace mrrocpp {
namespace edp {
namespace common {


#define MAX_INC 80
// #define MAX_INCR 469
// #define MAX_INCR2 234
// #define MAX_INCR3 30 // 15

#define ALGORITHM_AND_PARAMETERS_OK           0
#define UNIDENTIFIED_ALGORITHM_NO             1
#define UNIDENTIFIED_ALGORITHM_PARAMETERS_NO  2


// stale ograniczen na predkosc i przyspieszenie w regulatorach

const double SG_REG_1_MAX_SPEED = 0.4;
//const double SG_REG_1_MAX_ACC = 0.01;
const double SG_REG_1_MAX_ACC = 1;

const double SG_REG_2_MAX_SPEED = 0.4;
//const double SG_REG_2_MAX_ACC = 0.01;
const double SG_REG_2_MAX_ACC = 1;

const double SG_REG_3_MAX_SPEED = 0.4;
//const double SG_REG_3_MAX_ACC = 0.01;
const double SG_REG_3_MAX_ACC = 1;

const double SG_REG_4_MAX_SPEED = 0.4;
//const double SG_REG_4_MAX_ACC = 0.01;
const double SG_REG_4_MAX_ACC = 1;

const double SG_REG_5_MAX_SPEED = 0.4;
//const double SG_REG_5_MAX_ACC = 0.01;
const double SG_REG_5_MAX_ACC = 1;

const double SG_REG_6_MAX_SPEED = 0.4;
//const double SG_REG_6_MAX_ACC = 0.01;
const double SG_REG_6_MAX_ACC = 1;

const double SG_REG_7_MAX_SPEED = 1.0;
//const double SG_REG_7_MAX_ACC = 0.01;
const double SG_REG_7_MAX_ACC = 1;

const double SG_REG_8_MAX_SPEED = 1000.0;
const double SG_REG_8_MAX_ACC = 1000.0;

/*-----------------------------------------------------------------------*/
class regulator
{
    /* Klasa bazowa (i abstrakcyjna) dla poszczegolnych regulatorow konkretnych */

protected:

    // Zmienne lokalne klasy oraz funkcje wykorzystywane jedynie
    // wewnatrz tej klasy, tzn. przez algorytm regulacji.
    // Niezaleznie od zmiennych uzywanych przez algorytm konkretny
    // ponizsze zmienne musza byc zawsze aktualizowany, ze wzgledu na
    // "bezszelestne" przelaczanie algorytmow.

    int meassured_current; // wartosc zmierzona pradu

    double position_increment_old;  // przedosatnio odczytany przyrost polozenie (delta y[k-2]
    // -- mierzone w impulsach)
    double position_increment_new;  // ostatnio odczytany przyrost polozenie (delta y[k-1]
    // -- mierzone w impulsach)
    double pos_increment_new_sum;   // suma odczytanych przyrostow polozenia w trakcie realizacji makrokroku
    // (mierzona w impulsach)// by Y dla EDP_master
    double servo_pos_increment_new_sum;   // suma odczytanych przyrostow polozenia w trakcie realizacji makrokroku
    // (mierzona w impulsach)// by Y dla EDP_servo
    double step_old_pulse;                // poprzednia wartosc zadana dla jednego kroku regulacji
    // (przyrost wartosci zadanej polozenia -- delta r[k-2]
    // -- mierzone w radianach)
    double step_new;                // nastepna wartosc zadana dla jednego kroku regulacji
    // (przyrost wartosci zadanej polozenia -- delta r[k-1]
    // -- mierzone w radianach)
    double step_old;       // poprzednia wartosc zadana dla jednego kroku regulacji
    // (przyrost wartosci zadanej polozenia -- delta r[k-1]
    // -- mierzone w radianach)

    double step_new_over_constraint_sum;


    double set_value_new;           // wielkosc kroku do realizacji przez HI (wypelnienie PWM -- u[k])
    double set_value_old;           // wielkosc kroku do realizacji przez HI (wypelnienie PWM -- u[k-1])
    double set_value_very_old;      // wielkosc kroku do realizacji przez HI (wypelnienie PWM -- u[k-2])
    double delta_eint;              // przyrost calki uchybu
    double delta_eint_old;          // przyrost calki uchybu w poprzednim kroku


    int PWM_value;                  // zadane wypelnienie PWM
    uint8_t algorithm_no;              // przeslany numer algorytmu
    uint8_t current_algorithm_no;      // numer aktualnie uzywanego algorytmu
    uint8_t algorithm_parameters_no;   // przeslany numer zestawu parametrow algorytmu
    uint8_t current_algorithm_parameters_no;  // numer aktualnie uzywanego zestawu parametrow algorytmu

    lib::GRIPPER_STATE_ENUM reg_state, next_reg_state, prev_reg_state; // stany w ktorych moze byc regulator

    // BY Y i S - uwzglednie ograniczen na predkosc i przyspieszenie
    // stara wersja - nieuzywana
    // void constraint_detector(double max_acc_local, double max_vel_local, double max_diff_local, bool debug = false);
    // nowa wersja
    void constraint_detector(double max_acc_local, double max_vel_local, bool debug = false);

public:

    manip_and_conv_effector &master;
    regulator ( uint8_t reg_no, uint8_t reg_par_no,    manip_and_conv_effector &_master ); // konstruktor

    virtual uint8_t compute_set_value ( void ) = 0;
    // obliczenie nastepnej wartosci zadanej dla napedu - metoda abstrakcyjna

    double get_set_value ( void ) const;
    double previous_abs_position; // poprzednia pozycja absolutna dla potrzeb trybu testowego
    void insert_new_step (double ns);
    void insert_meassured_current (int meassured_current_l);

    double return_new_step () const;

    void insert_new_pos_increment (double inc);

    double get_position_inc ( int tryb );

    int get_meassured_current ( void ) const;

    int get_PWM_value ( void ) const;

    // do odczytu stanu regulatora (w szczegolnosci regulatora chwytaka)
    int get_reg_state ( void ) const;


    int get_actual_inc ( void ) const;

    // double get_desired_inc ( int axe_nr );

    void insert_algorithm_no ( uint8_t new_number );

    uint8_t get_algorithm_no ( void ) const;

    void insert_algorithm_parameters_no ( uint8_t new_number );

    uint8_t get_algorithm_parameters_no ( void ) const;

    void clear_regulator (void);
};
/*-----------------------------------------------------------------------*/



/*-----------------------------------------------------------------------*/
class NL_regulator: public regulator
{
    /* Klasa regulatorow konkretnych */
    // Obiekt z algorytmem regulacji

protected:
    // zmienne lokalne klasy oraz funkcje wykorzystywane jedynie
    //  wewnatrz tej klasy, tzn. przez algorytm regulacji
    double a, b0, b1;               // parametry regulatora
    double k_feedforward;           // wzmocnienie w petli "feedforward"
    double EPS;     // Dokladnosc zera dla przyrostow
    unsigned int integrator_off; // Liczba krokow zerowego PWM po ktorej wylaczmy calkowanie
    unsigned int counter;        // Licznik krokow zerowego PWM
    double MAX_PWM;

    //
    double int_current_error;
    int display;
    //

public:

    NL_regulator (uint8_t reg_no, uint8_t reg_par_no,
                  double aa, double bb0, double bb1, double k_ff,
                  manip_and_conv_effector &_master); // konstruktor

    virtual uint8_t compute_set_value ( void ) = 0;
    // obliczenie nastepnej wartosci zadanej dla napedu - metoda abstrakcyjna

};
// ----------------------------------------------------------------------





} // namespace common
} // namespace edp
} // namespace mrrocpp

#endif
