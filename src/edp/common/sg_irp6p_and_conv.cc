// ------------------------------------------------------------------------
//                                       edp.cc
//
// EDP_MASTER Effector Driver Master Process
// Driver dla robota IRp-6 na torze - metody: class edp_irp6s_and_conv_robot
//
// Ostatnia modyfikacja: styczen 2005
// -------------------------------------------------------------------------

#include <stdio.h>
#include <math.h>
#include <iostream>

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"
#include "edp/common/edp.h"
#include "edp/common/sg_irp6p_and_conv.h"

#include "lib/mathtr.h"

namespace mrrocpp {
namespace edp {
namespace common {

// uint64_t kk;	// numer pomiaru od momentu startu pomiarow


/*-----------------------------------------------------------------------*/
NL_regulator_1_irp6p::NL_regulator_1_irp6p (uint8_t reg_no, uint8_t reg_par_no, double aa, double bb0, double bb1, double k_ff, common::manip_and_conv_effector &_master)
        : NL_regulator(reg_no, reg_par_no, aa, bb0, bb1, k_ff, _master)
{
    // Konstruktor regulatora konkretnego
    // Przy inicjacji nalezy dopilnowac, zeby numery algorytmu regulacji oraz zestawu jego parametrow byly
    // zgodne z faktycznie przekazywanym zestawem parametrow inicjujacych.
}
/*-----------------------------------------------------------------------*/


/*-----------------------------------------------------------------------*/
NL_regulator_2_irp6p::NL_regulator_2_irp6p (uint8_t reg_no, uint8_t reg_par_no, double aa, double bb0, double bb1, double k_ff, common::manip_and_conv_effector &_master)
        : NL_regulator(reg_no, reg_par_no, aa, bb0, bb1, k_ff, _master)
{
    // Konstruktor regulatora konkretnego
    // Przy inicjacji nalezy dopilnowac, zeby numery algorytmu regulacji oraz zestawu jego parametrow byly
    // zgodne z faktycznie przekazywanym zestawem parametrow inicjujacych.
    int_current_error = 0;
    display = 0;
}
/*-----------------------------------------------------------------------*/


/*-----------------------------------------------------------------------*/
NL_regulator_3_irp6p::NL_regulator_3_irp6p (uint8_t reg_no, uint8_t reg_par_no, double aa, double bb0, double bb1, double k_ff, common::manip_and_conv_effector &_master)
        : NL_regulator(reg_no, reg_par_no, aa, bb0, bb1, k_ff, _master)
{
    // Konstruktor regulatora konkretnego
    // Przy inicjacji nalezy dopilnowac, zeby numery algorytmu regulacji oraz zestawu jego parametrow byly
    // zgodne z faktycznie przekazywanym zestawem parametrow inicjujacych.
    int_current_error = 0;
    display = 0;
}
/*-----------------------------------------------------------------------*/


/*-----------------------------------------------------------------------*/
NL_regulator_4_irp6p::NL_regulator_4_irp6p (uint8_t reg_no, uint8_t reg_par_no, double aa, double bb0, double bb1, double k_ff, common::manip_and_conv_effector &_master)
        : NL_regulator(reg_no, reg_par_no, aa, bb0, bb1, k_ff, _master)
{
    // Konstruktor regulatora konkretnego
    // Przy inicjacji nalezy dopilnowac, zeby numery algorytmu regulacji oraz zestawu jego parametrow byly
    // zgodne z faktycznie przekazywanym zestawem parametrow inicjujacych.
    int_current_error = 0;
    display = 0;
}
/*-----------------------------------------------------------------------*/


/*-----------------------------------------------------------------------*/
NL_regulator_5_irp6p::NL_regulator_5_irp6p (uint8_t reg_no, uint8_t reg_par_no, double aa, double bb0, double bb1, double k_ff, common::manip_and_conv_effector &_master)
        : NL_regulator(reg_no, reg_par_no, aa, bb0, bb1, k_ff, _master)
{
    // Konstruktor regulatora konkretnego
    // Przy inicjacji nalezy dopilnowac, zeby numery algorytmu regulacji oraz zestawu jego parametrow byly
    // zgodne z faktycznie przekazywanym zestawem parametrow inicjujacych.
    int_current_error = 0;
    display = 0;

    first = true;
}
/*-----------------------------------------------------------------------*/


/*-----------------------------------------------------------------------*/
NL_regulator_6_irp6p::NL_regulator_6_irp6p (uint8_t reg_no, uint8_t reg_par_no, double aa, double bb0, double bb1, double k_ff, common::manip_and_conv_effector &_master)
        : NL_regulator(reg_no, reg_par_no, aa, bb0, bb1, k_ff, _master)
{
    // Konstruktor regulatora konkretnego
    // Przy inicjacji nalezy dopilnowac, zeby numery algorytmu regulacji oraz zestawu jego parametrow byly
    // zgodne z faktycznie przekazywanym zestawem parametrow inicjujacych.
    int_current_error = 0;
    display = 0;
}
/*-----------------------------------------------------------------------*/


/*-----------------------------------------------------------------------*/
NL_regulator_7_irp6p::NL_regulator_7_irp6p (uint8_t reg_no, uint8_t reg_par_no, double aa, double bb0, double bb1, double k_ff, common::manip_and_conv_effector &_master)
        : NL_regulator(reg_no, reg_par_no, aa, bb0, bb1, k_ff, _master)
{
    // Konstruktor regulatora konkretnego
    // Przy inicjacji nalezy dopilnowac, zeby numery algorytmu regulacji oraz zestawu jego parametrow byly
    // zgodne z faktycznie przekazywanym zestawem parametrow inicjujacych.
    int_current_error = 0;
    display = 0;
}
/*-----------------------------------------------------------------------*/


/*-----------------------------------------------------------------------*/
NL_regulator_8_irp6p::NL_regulator_8_irp6p (uint8_t reg_no, uint8_t reg_par_no, double aa, double bb0, double bb1, double k_ff, common::manip_and_conv_effector &_master)
        : NL_regulator(reg_no, reg_par_no, aa, bb0, bb1, k_ff, _master)
{
    reg_state = next_reg_state = prev_reg_state = lib::GRIPPER_START_STATE;
    sum_of_currents = current_index = 0;
    for (int i=0; i < IRP6_POSTUMENT_GRIPPER_SUM_OF_CURRENTS_NR_OF_ELEMENTS; i++)
    {
        currents [i] = 0;
    }
    display=0;

    // Konstruktor regulatora konkretnego
    // Przy inicjacji nalezy dopilnowac, zeby numery algorytmu regulacji oraz zestawu jego parametrow byly
    // zgodne z faktycznie przekazywanym zestawem parametrow inicjujacych.
}
/*-----------------------------------------------------------------------*/


// tasmociag


/*-----------------------------------------------------------------------*/
uint8_t NL_regulator_1_irp6p::compute_set_value (void)
{
    // algorytm regulacji dla serwomechanizmu
    // position_increment_old - przedostatnio odczytany przyrost polozenie
    //                         (delta y[k-2] -- mierzone w impulsach)
    // position_increment_new - ostatnio odczytany przyrost polozenie
    //                         (delta y[k-1] -- mierzone w impulsach)
    // step_old_pulse               - poprzednia wartosc zadana dla jednego kroku
    //                         regulacji (przyrost wartosci zadanej polozenia --
    //                         delta r[k-2] -- mierzone w impulsach)
    // step_new               - nastepna wartosc zadana dla jednego kroku
    //                         regulacji (przyrost wartosci zadanej polozenia --
    //                         delta r[k-1] -- mierzone w radianach)
    // set_value_new          - wielkosc kroku do realizacji przez HIP
    //                         (wypelnienie PWM -- u[k]): czas trwania jedynki
    // set_value_old          - wielkosc kroku do realizacji przez HIP
    //                         (wypelnienie PWM -- u[k-1]): czas trwania jedynki
    // set_value_very_old     - wielkosc kroku do realizacji przez HIP
    //                         (wypelnienie PWM -- u[k-2]): czas trwania jedynki

    double step_new_pulse; // nastepna wartosc zadana dla jednego kroku regulacji
    // (przyrost wartosci zadanej polozenia --
    // delta r[k-1] -- mierzone w impulsach)
    uint8_t alg_par_status;   // okresla prawidlowosc numeru algorytmu regulacji
    // i zestawu jego parametrow



    alg_par_status = ALGORITHM_AND_PARAMETERS_OK;

    // double root_position_increment_new=position_increment_new;

    // BY Y i S - uwzglednie ograniczen na predkosc i przyspieszenie
    constraint_detector(common::SG_REG_1_MAX_ACC, common::SG_REG_1_MAX_SPEED);

    // przeliczenie radianow na impulsy
    // step_new_pulse = step_new*IRP6_POSTUMENT_INC_PER_REVOLUTION/(2*M_PI); // ORIGINAL
    step_new_pulse = step_new*IRP6_POSTUMENT_AXIS_0_TO_5_INC_PER_REVOLUTION/(2*M_PI);
    //position_increment_new=position_increment_new/AXE_0_TO_5_POSTUMENT_TO_TRACK_RATIO;

    /*
    if (!aaa)
     if ( (fabs(step_new_pulse) < 0.0001) && (fabs(position_increment_new ) > 1) ) {
      aaa++;
     }
     */
    // if (aaa > 0 && aaa < 30 ) {
    //  cprintf("O1: svn=%4.0lf svo=%4.0lf svvo=%4.0lf de=%4.0lf deo=%4.0lf snp=%4.0lf pin=%4.0lf pio=%4.0lf\n",set_value_new,set_value_old,
    //  set_value_very_old, delta_eint, delta_eint_old, step_new_pulse,position_increment_new,position_increment_old);
    //  cprintf("O1: snp=%4.0lf pin=%4.0lf pio=%4.0lf L=%4x U=%4x\n", step_new_pulse,position_increment_new,position_increment_old,md.robot_status[0].adr_224,md.robot_status[0].adr_226);
    //  aaa++;
    //  if (aaa == 9) aaa=0;
    // }

    /* // by Y - bez sensu
    // Jesli rzeczywisty przyrost jest wiekszy od dopuszczalnego
     if (fabs(position_increment_new) > MAX_INC)
       position_increment_new = position_increment_old;
    */

    // kumulacja przyrostu polozenia w tym makrokroku // ORIGINAL
    // pos_increment_new_sum += position_increment_new*POSTUMENT_TO_TRACK_RATIO;
    // servo_pos_increment_new_sum += position_increment_new*POSTUMENT_TO_TRACK_RATIO; // by Y

    // kumulacja przyrostu polozenia w tym makrokroku
    // pos_increment_new_sum += root_position_increment_new;
    // servo_pos_increment_new_sum += root_position_increment_new;// by Y


    // Przyrost calki uchybu
    delta_eint = delta_eint_old + 1.008*(step_new_pulse- position_increment_new) -
                 0.992*(step_old_pulse - position_increment_old);

    // if (fabs(step_new_pulse) > 70.0) {
    //  cprintf("snp = %lf   pin = %lf\n",step_new_pulse, position_increment_new);
    // }

    // if (fabs(delta_eint) > 50.0) {
    //  cprintf("%4.0lf ",delta_eint);
    // }

    // Sprawdzenie czy numer algorytmu lub zestawu parametrow sie zmienil?
    // Jezeli tak, to nalezy dokonac uaktualnienia numerow (ewentualnie wykryc niewlasciwosc numerow)
    if ( (current_algorithm_no != algorithm_no) ||
            (current_algorithm_parameters_no != algorithm_parameters_no) )
    {
        switch (algorithm_no)
        {
        case 0:  // algorytm nr 0
            switch (algorithm_parameters_no)
            {
            case 0:  // zestaw parametrow nr 0
                current_algorithm_parameters_no = algorithm_parameters_no;
                current_algorithm_no = algorithm_no;
                a  = 0.4152;
                b0 = 0.9017*1.5;
                b1 = 0.7701*1.5;
                k_feedforward = 0.35;
                break;
            case 1: // zestaw parametrow nr 1
                current_algorithm_parameters_no = algorithm_parameters_no;
                current_algorithm_no = algorithm_no;
                a  = 0.4152;
                b0 = 0.9017*1.0;
                b1 = 0.7701*1.0;
                k_feedforward = 0;
                break;
            default: // blad => przywrocic stary algorytm i j stary zestaw parametrow
                algorithm_no = current_algorithm_no;
                algorithm_parameters_no = current_algorithm_parameters_no;
                alg_par_status = UNIDENTIFIED_ALGORITHM_PARAMETERS_NO;
                break;
            }
            break;
        case 1: // algorytm nr 1
            switch (algorithm_parameters_no)
            {
            case 0: // zestaw parametrow nr 0
                current_algorithm_parameters_no = algorithm_parameters_no;
                current_algorithm_no = algorithm_no;
                a = 0;
                b0 = 0;
                b1 = 0;
                k_feedforward = 0;
                break;
            case 1: // zestaw parametrow nr 1
                current_algorithm_parameters_no = algorithm_parameters_no;
                current_algorithm_no = algorithm_no;
                a = 0;
                b0 = 0;
                b1 = 0;
                k_feedforward = 0;
                break;
            default: // blad - nie ma takiego zestawu parametrow dla tego algorytmu
                // => przywrocic stary algorytm i j stary zestaw parametrow
                algorithm_no = current_algorithm_no;
                algorithm_parameters_no = current_algorithm_parameters_no;
                alg_par_status = UNIDENTIFIED_ALGORITHM_PARAMETERS_NO;
                break;
            }
            ; // end: switch (algorithm_parameters_no)
            break;
        default: // blad - nie ma takiego algorytmu
            // => przywrocic stary algorytm i j stary zestaw parametrow
            algorithm_no = current_algorithm_no;
            algorithm_parameters_no = current_algorithm_parameters_no;
            alg_par_status = UNIDENTIFIED_ALGORITHM_NO;
            break;
        }
        ;  // end: switch (algorithm_no)
    }



    a=0.548946716233;
    b0=1.576266; //9.244959545156;
    b1=1.468599; //8.613484947882;


    switch (algorithm_no)
    {
    case 0:  // algorytm nr 0
        // obliczenie nowej wartosci wypelnienia PWM algorytm PD + I
        set_value_new = (1+a)*set_value_old - a*set_value_very_old +
                        b0*delta_eint - b1*delta_eint_old;
        if ( ( fabs(set_value_new)) < 0.1 )
            counter++;
        else
            counter = 0;

        if ( fabs(step_new) < EPS && fabs(position_increment_new) < EPS && (counter > integrator_off) )
        {
            set_value_new = (1+a)*set_value_old - a*set_value_very_old +
                            b0*(step_new_pulse - position_increment_new)
                            - b1*(step_old_pulse - position_increment_old);
        }
        break;
    case 1:  // algorytm nr 1
        // obliczenie nowej wartosci wypelnienia PWM algorytm PD + I
        set_value_new = (1+a)*set_value_old - a*set_value_very_old +
                        b0*(step_new_pulse - position_increment_new)
                        - b1*(step_old_pulse - position_increment_old);
        break;
    default: // w tym miejscu nie powinien wystapic blad zwiazany z
        // nieistniejacym numerem algorytmu
        set_value_new = 0; // zerowe nowe sterowanie
        break;
    }

    master.rb_obj->lock_mutex();

    master.rb_obj->step_data.desired_inc[0] = (float) step_new_pulse; // pozycja osi 0
    master.rb_obj->step_data.current_inc[0] = (short int) position_increment_new;
    master.rb_obj->step_data.pwm[0] = (float) set_value_new;
    master.rb_obj->step_data.uchyb[0]=(float) (step_new_pulse - position_increment_new);

    master.rb_obj->unlock_mutex();

    // ograniczenie na sterowanie
    if (set_value_new > MAX_PWM)
        set_value_new = MAX_PWM;
    if (set_value_new < -MAX_PWM)
        set_value_new = -MAX_PWM;

    // ograniczenie przyrostu PWM
    // ma na celu zapobiegac osiaganiu zbyt duzych pradow we wzmacniaczach mocy
    if (set_value_new - set_value_old > CONVEYOR_AXE1_MAX_PWM_INCREMENT)
        set_value_new = set_value_old + CONVEYOR_AXE1_MAX_PWM_INCREMENT;
    if (set_value_new- set_value_old < -CONVEYOR_AXE1_MAX_PWM_INCREMENT)
        set_value_new = set_value_old - CONVEYOR_AXE1_MAX_PWM_INCREMENT;

    // przepisanie nowych wartosci zmiennych do zmiennych przechowujacych wartosci poprzednie
    position_increment_old = position_increment_new;
    delta_eint_old = delta_eint;
    step_old_pulse = step_new_pulse;
    set_value_very_old = set_value_old;
    set_value_old = set_value_new;
    PWM_value = (int) set_value_new;

    return alg_par_status;

}
/*-----------------------------------------------------------------------*/


// kolumna i tak dalej
/*-----------------------------------------------------------------------*/
uint8_t NL_regulator_2_irp6p::compute_set_value (void)
{
    // algorytm regulacji dla serwomechanizmu

    // position_increment_old - przedostatnio odczytany przyrost polozenie
    //                         (delta y[k-2] -- mierzone w impulsach)
    // position_increment_new - ostatnio odczytany przyrost polozenie
    //                         (delta y[k-1] -- mierzone w impulsach)
    // step_old_pulse               - poprzednia wartosc zadana dla jednego kroku
    //                         regulacji (przyrost wartosci zadanej polozenia --
    //                         delta r[k-2] -- mierzone w impulsach)
    // step_new               - nastepna wartosc zadana dla jednego kroku
    //                         regulacji (przyrost wartosci zadanej polozenia --
    //                         delta r[k-1] -- mierzone w radianach)
    // set_value_new          - wielkosc kroku do realizacji przez HIP
    //                         (wypelnienie PWM -- u[k]): czas trwania jedynki
    // set_value_old          - wielkosc kroku do realizacji przez HIP
    //                         (wypelnienie PWM -- u[k-1]): czas trwania jedynki
    // set_value_very_old     - wielkosc kroku do realizacji przez HIP
    //                         (wypelnienie PWM -- u[k-2]): czas trwania jedynki

    double step_new_pulse; // nastepna wartosc zadana dla jednego kroku regulacji
    // (przyrost wartosci zadanej polozenia --
    // delta r[k-1] -- mierzone w impulsach)
    uint8_t alg_par_status;   // okresla prawidlowosc numeru algorytmu regulacji
    // i zestawu jego parametrow
    //
    double current_error;
    double current_desired;
    double current_measured;
    //

    alg_par_status = ALGORITHM_AND_PARAMETERS_OK;

    // double root_position_increment_new=position_increment_new;

    // BY Y i S - uwzglednie ograniczen na predkosc i przyspieszenie
    constraint_detector(common::SG_REG_2_MAX_ACC, common::SG_REG_2_MAX_SPEED);

    // przeliczenie radianow na impulsy
    // step_new_pulse = step_new*IRP6_POSTUMENT_INC_PER_REVOLUTION/(2*M_PI); // ORIGINAL
    step_new_pulse = step_new*IRP6_POSTUMENT_AXIS_0_TO_5_INC_PER_REVOLUTION/(2*M_PI);
    //position_increment_new= position_increment_new/AXE_0_TO_5_POSTUMENT_TO_TRACK_RATIO;

    // if (step_new!=0.0) printf(" 2 reg:%f\n", step_new);

    /*
    if (!bbb)
     if ( (fabs(step_new_pulse) < 0.0001) && (fabs(position_increment_new - position_increment_old ) > 40) ) {
      bbb++;
     }
     */
    // if (bbb > 0 && bbb < 10 ) {
    //  cprintf("O2: svn=%4.0lf svo=%4.0lf svvo=%4.0lf de=%4.0lf deo=%4.0lf snp=%4.0lf pin=%4.0lf pio=%4.0lf\n",set_value_new,set_value_old,
    //  set_value_very_old, delta_eint, delta_eint_old, step_new_pulse,position_increment_new,position_increment_old);
    //  cprintf("O2: snp=%4.0lf pin=%4.0lf pio=%4.0lf L=%4x U=%4x\n", step_new_pulse,position_increment_new,position_increment_old,md.robot_status[1].adr_224,md.robot_status[1].adr_226);
    //  bbb++;
    //  if (bbb == 9) bbb=0;
    // }
    /* // by Y - bez sensu
    // Jesli rzeczywisty przyrost jest wiekszy od dopuszczalnego
     if (fabs(position_increment_new) > MAX_INC)
       position_increment_new = position_increment_old;
    */

    // kumulacja przyrostu polozenia w tym makrokroku // ORIGINAL
    // pos_increment_new_sum += position_increment_new*POSTUMENT_TO_TRACK_RATIO;
    // servo_pos_increment_new_sum += position_increment_new*POSTUMENT_TO_TRACK_RATIO; // by Y

    // kumulacja przyrostu polozenia w tym makrokroku
    //pos_increment_new_sum += root_position_increment_new;
    //servo_pos_increment_new_sum += root_position_increment_new;// by Y

    // kumulacja przyrostu polozenia w tym makrokroku
    // pos_increment_new_sum += position_increment_new;
    // servo_pos_increment_new_sum += position_increment_new;// by Y

    // Przyrost calki uchybu
    delta_eint = delta_eint_old + 1.010*(step_new_pulse - position_increment_new) -
                 0.990*(step_old_pulse - position_increment_old);

    // Sprawdzenie czy numer algorytmu lub zestawu parametrow sie zmienil?
    // Jezeli tak, to nalezy dokonac uaktualnienia numerow (ewentualnie wykryc niewlasciwosc numerow)
    if ( (current_algorithm_no != algorithm_no) ||
            (current_algorithm_parameters_no != algorithm_parameters_no) )
    {
        switch (algorithm_no)
        {
        case 0:  // algorytm nr 0
            switch (algorithm_parameters_no)
            {
            case 0:  // zestaw parametrow nr 0
                current_algorithm_parameters_no = algorithm_parameters_no;
                current_algorithm_no = algorithm_no;
                a  = 0.3079;
                b0 = 2.3100*1.5;
                b1 = 2.0312*1.5;
                k_feedforward = 0.35;
                break;
            case 1: // zestaw parametrow nr 1
                current_algorithm_parameters_no = algorithm_parameters_no;
                current_algorithm_no = algorithm_no;
                a  = 0.3079;
                b0 = 2.3100*2.0;
                b1 = 2.0312*2.0;
                k_feedforward = 0;
                break;
            default: // blad => przywrocic stary algorytm i j stary zestaw parametrow
                algorithm_no = current_algorithm_no;
                algorithm_parameters_no = current_algorithm_parameters_no;
                alg_par_status = UNIDENTIFIED_ALGORITHM_PARAMETERS_NO;
                break;
            }
            break;
        case 1: // algorytm nr 1
            switch (algorithm_parameters_no)
            {
            case 0: // zestaw parametrow nr 0
                current_algorithm_parameters_no = algorithm_parameters_no;
                current_algorithm_no = algorithm_no;
                a = 0;
                b0 = 0;
                b1 = 0;
                k_feedforward = 0;
                break;
            case 1: // zestaw parametrow nr 1
                current_algorithm_parameters_no = algorithm_parameters_no;
                current_algorithm_no = algorithm_no;
                a = 0;
                b0 = 0;
                b1 = 0;
                k_feedforward = 0;
                break;
            default: // blad - nie ma takiego zestawu parametrow dla tego algorytmu
                // => przywrocic stary algorytm i j stary zestaw parametrow
                algorithm_no = current_algorithm_no;
                algorithm_parameters_no = current_algorithm_parameters_no;
                alg_par_status = UNIDENTIFIED_ALGORITHM_PARAMETERS_NO;
                break;
            }
            ; // end: switch (algorithm_parameters_no)
            break;
        case 2:  // algorytm nr 2 - sterowanie pradowe
            current_algorithm_parameters_no = algorithm_parameters_no;
            current_algorithm_no = algorithm_no;
            break;
        default: // blad - nie ma takiego algorytmu
            // => przywrocic stary algorytm i j stary zestaw parametrow
            algorithm_no = current_algorithm_no;
            algorithm_parameters_no = current_algorithm_parameters_no;
            alg_par_status = UNIDENTIFIED_ALGORITHM_NO;
            break;
        }
    }

    a=0.412429378531;
    b0=2.594932; //stara z przelicz rezolwer/enkoder 15.219541375872
    b1=2.504769; //

    switch (algorithm_no)
    {
    case 0:  // algorytm nr 0
        // obliczenie nowej wartosci wypelnienia PWM algorytm PD + I
        set_value_new = (1+a)*set_value_old - a*set_value_very_old +
                        b0*delta_eint - b1*delta_eint_old;
        // cout<<a<<" "<<b0<<" "<<b1<<"\n";
        break;
    case 1:  // algorytm nr 1
        // obliczenie nowej wartosci wypelnienia PWM algorytm PD + I
        set_value_new = (1+a)*set_value_old - a*set_value_very_old +
                        b0*(step_new_pulse - position_increment_new)
                        - b1*(step_old_pulse - position_increment_old);
        break;
    case 2:  // algorytm nr 2 - sterowanie pradowe
        // DUNG START
        current_desired = 0.4*9.52 * (master.new_instruction.arm.pf_def.desired_torque[0] / 158);
        current_measured = (meassured_current - 128 - 3)*0.035;
        current_error = current_desired - current_measured;
        int_current_error =  int_current_error + current_error*0.02;		// 500Hz => 0.02s
        //int_current_error = 0;
        set_value_new = -30 * current_error - 6.0*int_current_error;

        display++;
        if (display >= 500)
        {
            display = 0;
            //printf("joint 1:   desired_current = %f,    current_error = %f,    out = %f\n", master.new_instruction.arm.pf_def.desired_torque[0] / 158, current_error, -30 * current_error - 4.0*int_current_error);
        }
        // DUNG END
        break;
    default: // w tym miejscu nie powinien wystapic blad zwiazany z
        // nieistniejacym numerem algorytmu
        set_value_new = 0; // zerowe nowe sterowanie
        break;
    }


    master.rb_obj->lock_mutex();


    master.rb_obj->step_data.desired_inc[1] = (float) step_new_pulse; // pozycja osi 0
    master.rb_obj->step_data.current_inc[1] = (short int) position_increment_new;
    master.rb_obj->step_data.pwm[1] = (float) set_value_new;
    master.rb_obj->step_data.uchyb[1]=(float) (step_new_pulse - position_increment_new);
    master.rb_obj->unlock_mutex();


    //  	set_value_new=set_value_new;

    // ograniczenie na sterowanie
    if (set_value_new > MAX_PWM)
        set_value_new = MAX_PWM;
    if (set_value_new < -MAX_PWM)
        set_value_new = -MAX_PWM;

    // ograniczenie przyrostu PWM
    // ma na celu zapobiegac osiaganiu zbyt duzych pradow we wzmacniaczach mocy
    if (set_value_new - set_value_old > IRP6_POSTUMENT_AXE2_MAX_PWM_INCREMENT)
        set_value_new = set_value_old + IRP6_POSTUMENT_AXE2_MAX_PWM_INCREMENT;
    if (set_value_new- set_value_old < -IRP6_POSTUMENT_AXE2_MAX_PWM_INCREMENT)
        set_value_new = set_value_old - IRP6_POSTUMENT_AXE2_MAX_PWM_INCREMENT;

    // przepisanie nowych wartosci zmiennych do zmiennych przechowujacych wartosci poprzednie
    position_increment_old = position_increment_new;
    delta_eint_old = delta_eint;
    step_old_pulse = step_new_pulse;
    set_value_very_old = set_value_old;
    set_value_old = set_value_new;
    PWM_value = (int) set_value_new;

    return alg_par_status;

}
/*-----------------------------------------------------------------------*/


/*-----------------------------------------------------------------------*/
uint8_t NL_regulator_3_irp6p::compute_set_value (void)
{
    // algorytm regulacji dla serwomechanizmu

    // position_increment_old - przedostatnio odczytany przyrost polozenie
    //                         (delta y[k-2] -- mierzone w impulsach)
    // position_increment_new - ostatnio odczytany przyrost polozenie
    //                         (delta y[k-1] -- mierzone w impulsach)
    // step_old_pulse               - poprzednia wartosc zadana dla jednego kroku
    //                         regulacji (przyrost wartosci zadanej polozenia --
    //                         delta r[k-2] -- mierzone w impulsach)
    // step_new               - nastepna wartosc zadana dla jednego kroku
    //                         regulacji (przyrost wartosci zadanej polozenia --
    //                         delta r[k-1] -- mierzone w radianach)
    // set_value_new          - wielkosc kroku do realizacji przez HIP
    //                         (wypelnienie PWM -- u[k]): czas trwania jedynki
    // set_value_old          - wielkosc kroku do realizacji przez HIP
    //                         (wypelnienie PWM -- u[k-1]): czas trwania jedynki
    // set_value_very_old     - wielkosc kroku do realizacji przez HIP
    //                         (wypelnienie PWM -- u[k-2]): czas trwania jedynki

    double step_new_pulse; // nastepna wartosc zadana dla jednego kroku regulacji
    // (przyrost wartosci zadanej polozenia --
    // delta r[k-1] -- mierzone w impulsach)
    uint8_t alg_par_status;   // okresla prawidlowosc numeru algorytmu regulacji
    // i zestawu jego parametrow
    //
    double current_error;
    double current_desired;
    double current_measured;
    //

    alg_par_status = ALGORITHM_AND_PARAMETERS_OK;

    // double root_position_increment_new=position_increment_new;

    // BY Y i S - uwzglednie ograniczen na predkosc i przyspieszenie
    constraint_detector(common::SG_REG_3_MAX_ACC, common::SG_REG_3_MAX_SPEED);

    // przeliczenie radianow na impulsy
    // step_new_pulse = step_new*IRP6_POSTUMENT_INC_PER_REVOLUTION/(2*M_PI); // ORIGINAL
    step_new_pulse = step_new*IRP6_POSTUMENT_AXIS_0_TO_5_INC_PER_REVOLUTION/(2*M_PI);
    ///position_increment_new= position_increment_new;

    // if (step_new!=0.0) printf(" 3 reg:%f\n", step_new);


    /*
    if (!ccc)
     if ( (fabs(step_new_pulse) < 0.0001) && (fabs(position_increment_new - position_increment_old ) > 40) ) {
      ccc++;
     }
     */

    // if (ccc > 0 && ccc < 10 ) {
    //  cprintf("O3: svn=%4.0lf svo=%4.0lf svvo=%4.0lf de=%4.0lf deo=%4.0lf snp=%4.0lf pin=%4.0lf pio=%4.0lf\n",set_value_new,set_value_old,
    //  set_value_very_old, delta_eint, delta_eint_old, step_new_pulse,position_increment_new,position_increment_old);
    //  cprintf("O3: snp=%4.0lf pin=%4.0lf pio=%4.0lf L=%4x U=%4x\n", step_new_pulse,position_increment_new,position_increment_old,md.robot_status[2].adr_224,md.robot_status[2].adr_226);
    //  ccc++;
    //  if (ccc == 9) ccc=0;
    // }

    /* // by Y - bez sensu
    // Jesli rzeczywisty przyrost jest wiekszy od dopuszczalnego
     if (fabs(position_increment_new) > MAX_INC)
       position_increment_new = position_increment_old;
    */

    // kumulacja przyrostu polozenia w tym makrokroku // ORIGINAL
    // pos_increment_new_sum += position_increment_new*POSTUMENT_TO_TRACK_RATIO;
    // servo_pos_increment_new_sum += position_increment_new*POSTUMENT_TO_TRACK_RATIO; // by Y

    // kumulacja przyrostu polozenia w tym makrokroku
    // pos_increment_new_sum += root_position_increment_new;
    // servo_pos_increment_new_sum += root_position_increment_new;// by Y

    // Przyrost calki uchybu
    delta_eint = delta_eint_old + 1.008*(step_new_pulse - position_increment_new) -
                 0.992*(step_old_pulse - position_increment_old);

    // Sprawdzenie czy numer algorytmu lub zestawu parametrow sie zmienil?
    // Jezeli tak, to nalezy dokonac uaktualnienia numerow (ewentualnie wykryc niewlasciwosc numerow)
    if ( (current_algorithm_no != algorithm_no) ||
            (current_algorithm_parameters_no != algorithm_parameters_no) )
    {
        switch (algorithm_no)
        {
        case 0:  // algorytm nr 0
            switch (algorithm_parameters_no)
            {
            case 0:  // zestaw parametrow nr 0
                current_algorithm_parameters_no = algorithm_parameters_no;
                current_algorithm_no = algorithm_no;
                a  = 0.4152;
                b0 = 1.2500*1.5;
                b1 = 1.0998*1.5;
                k_feedforward = 0.35;
                break;
            case 1: // zestaw parametrow nr 1
                current_algorithm_parameters_no = algorithm_parameters_no;
                current_algorithm_no = algorithm_no;
                a  = 0.4152;
                b0 = 1.2500*2.5;
                b1 = 1.0998*2.5;
                k_feedforward = 0;
                break;
            default: // blad => przywrocic stary algorytm i j stary zestaw parametrow
                algorithm_no = current_algorithm_no;
                algorithm_parameters_no = current_algorithm_parameters_no;
                alg_par_status = UNIDENTIFIED_ALGORITHM_PARAMETERS_NO;
                break;
            }
            break;
        case 1: // algorytm nr 1
            switch (algorithm_parameters_no)
            {
            case 0: // zestaw parametrow nr 0
                current_algorithm_parameters_no = algorithm_parameters_no;
                current_algorithm_no = algorithm_no;
                a = 0;
                b0 = 0;
                b1 = 0;
                k_feedforward = 0;
                break;
            case 1: // zestaw parametrow nr 1
                current_algorithm_parameters_no = algorithm_parameters_no;
                current_algorithm_no = algorithm_no;
                a = 0;
                b0 = 0;
                b1 = 0;
                k_feedforward = 0;
                break;
            default: // blad - nie ma takiego zestawu parametrow dla tego algorytmu
                // => przywrocic stary algorytm i j stary zestaw parametrow
                algorithm_no = current_algorithm_no;
                algorithm_parameters_no = current_algorithm_parameters_no;
                alg_par_status = UNIDENTIFIED_ALGORITHM_PARAMETERS_NO;
                break;
            }
            ; // end: switch (algorithm_parameters_no)
            break;
        case 2:  // algorytm nr 2 - sterowanie pradowe
            current_algorithm_parameters_no = algorithm_parameters_no;
            current_algorithm_no = algorithm_no;
            break;
        default: // blad - nie ma takiego algorytmu
            // => przywrocic stary algorytm i j stary zestaw parametrow
            algorithm_no = current_algorithm_no;
            algorithm_parameters_no = current_algorithm_parameters_no;
            alg_par_status = UNIDENTIFIED_ALGORITHM_NO;
            break;
        }
    }

    a=0.655629139073;
    b0=1.030178; //6.042100283822;
    b1=0.986142;

    switch (algorithm_no)
    {
    case 0:  // algorytm nr 0
        // obliczenie nowej wartosci wypelnienia PWM algorytm PD + I
        set_value_new = (1+a)*set_value_old - a*set_value_very_old +
                        b0*delta_eint - b1*delta_eint_old;
        break;
    case 1:  // algorytm nr 1
        // obliczenie nowej wartosci wypelnienia PWM algorytm PD + I
        set_value_new = (1+a)*set_value_old - a*set_value_very_old +
                        b0*(step_new_pulse - position_increment_new)
                        - b1*(step_old_pulse - position_increment_old);
        break;
    case 2:  // algorytm nr 2 - sterowanie pradowe
        // DUNG START
        current_desired = 0.4*9.52 * (master.new_instruction.arm.pf_def.desired_torque[1] / 158);
        current_measured = (meassured_current - 128 - 3)*0.035;
        current_error = current_desired - current_measured;
        int_current_error =  int_current_error + current_error*0.02;		// 500Hz => 0.02s
        //int_current_error = 0;
        set_value_new = -33 * current_error - 6.4*int_current_error;

        display++;
        if (display >= 500)
        {
            display = 0;
            //printf("joint 2:   desired_current = %f(A),    current_error = %f(A),    out = %f\n", current_desired, current_error, -30 * current_error - 6*int_current_error);
        }
        // DUNG END
        break;
    default: // w tym miejscu nie powinien wystapic blad zwiazany z
        // nieistniejacym numerem algorytmu
        set_value_new = 0; // zerowe nowe sterowanie
        break;
    }


    master.rb_obj->lock_mutex();

    master.rb_obj->step_data.desired_inc[2] = (float) step_new_pulse; // pozycja osi 0
    master.rb_obj->step_data.current_inc[2] = (short int) position_increment_new;
    master.rb_obj->step_data.pwm[2] = (float) set_value_new;
    master.rb_obj->step_data.uchyb[2]=(float) (step_new_pulse - position_increment_new);

    master.rb_obj->unlock_mutex();

    // ograniczenie na sterowanie
    if (set_value_new > MAX_PWM)
        set_value_new = MAX_PWM;
    if (set_value_new < -MAX_PWM)
        set_value_new = -MAX_PWM;

    // ograniczenie przyrostu PWM
    // ma na celu zapobiegac osiaganiu zbyt duzych pradow we wzmacniaczach mocy
    if (set_value_new - set_value_old > IRP6_POSTUMENT_AXE3_MAX_PWM_INCREMENT)
        set_value_new = set_value_old + IRP6_POSTUMENT_AXE3_MAX_PWM_INCREMENT;
    if (set_value_new- set_value_old < -IRP6_POSTUMENT_AXE3_MAX_PWM_INCREMENT)
        set_value_new = set_value_old - IRP6_POSTUMENT_AXE3_MAX_PWM_INCREMENT;

    // przepisanie nowych wartosci zmiennych do zmiennych przechowujacych wartosci poprzednie
    position_increment_old = position_increment_new;
    delta_eint_old = delta_eint;
    step_old_pulse = step_new_pulse;
    set_value_very_old = set_value_old;
    set_value_old = set_value_new;
    PWM_value = (int) set_value_new;

    return alg_par_status;

}
/*-----------------------------------------------------------------------*/


/*-----------------------------------------------------------------------*/
uint8_t NL_regulator_4_irp6p::compute_set_value (void)
{
    // algorytm regulacji dla serwomechanizmu

    // position_increment_old - przedostatnio odczytany przyrost polozenie
    //                         (delta y[k-2] -- mierzone w impulsach)
    // position_increment_new - ostatnio odczytany przyrost polozenie
    //                         (delta y[k-1] -- mierzone w impulsach)
    // step_old_pulse               - poprzednia wartosc zadana dla jednego kroku
    //                         regulacji (przyrost wartosci zadanej polozenia --
    //                         delta r[k-2] -- mierzone w impulsach)
    // step_new               - nastepna wartosc zadana dla jednego kroku
    //                         regulacji (przyrost wartosci zadanej polozenia --
    //                         delta r[k-1] -- mierzone w radianach)
    // set_value_new          - wielkosc kroku do realizacji przez HIP
    //                         (wypelnienie PWM -- u[k]): czas trwania jedynki
    // set_value_old          - wielkosc kroku do realizacji przez HIP
    //                         (wypelnienie PWM -- u[k-1]): czas trwania jedynki
    // set_value_very_old     - wielkosc kroku do realizacji przez HIP
    //                         (wypelnienie PWM -- u[k-2]): czas trwania jedynki

    double step_new_pulse; // nastepna wartosc zadana dla jednego kroku regulacji
    // (przyrost wartosci zadanej polozenia --
    // delta r[k-1] -- mierzone w impulsach)
    uint8_t alg_par_status;   // okresla prawidlowosc numeru algorytmu regulacji
    // i zestawu jego parametrow

    alg_par_status = ALGORITHM_AND_PARAMETERS_OK;

    //
    double current_error;
    double current_desired;
    double current_measured;
    //


    // double root_position_increment_new=position_increment_new;

    // BY Y i S - uwzglednie ograniczen na predkosc i przyspieszenie
    // constraint_detector(common::SG_REG_4_MAX_ACC, common::SG_REG_4_MAX_SPEED);

    // przeliczenie radianow na impulsy
    // step_new_pulse = step_new*IRP6_POSTUMENT_INC_PER_REVOLUTION/(2*M_PI); // ORIGINAL
    step_new_pulse = step_new*IRP6_POSTUMENT_AXIS_0_TO_5_INC_PER_REVOLUTION/(2*M_PI);
    //position_increment_new= position_increment_new/AXE_0_TO_5_POSTUMENT_TO_TRACK_RATIO;

    // if (step_new!=0.0) printf(" 4 reg:%f\n", step_new);

    //	printf("joint 3: %d   \n", meassured_current-128);

    /*
    if (!ddd)
     if ( (fabs(step_new_pulse) < 0.0001) && (fabs(position_increment_new - position_increment_old ) > 40) ) {
      ddd++;
     }
     */
    // if (ddd > 0 && ddd < 10 ) {
    //  cprintf("O4: svn=%4.0lf svo=%4.0lf svvo=%4.0lf de=%4.0lf deo=%4.0lf snp=%4.0lf pin=%4.0lf pio=%4.0lf\n",set_value_new,set_value_old,
    //  set_value_very_old, delta_eint, delta_eint_old, step_new_pulse,position_increment_new,position_increment_old);
    //  cprintf("O4: snp=%4.0lf pin=%4.0lf pio=%4.0lf L=%4x U=%4x\n", step_new_pulse,position_increment_new,position_increment_old,md.robot_status[3].adr_224,md.robot_status[3].adr_226);
    //  ddd++;
    //  if (ddd == 9) ddd=0;
    // }

    /* // by Y - bez sensu
    // Jesli rzeczywisty przyrost jest wiekszy od dopuszczalnego
     if (fabs(position_increment_new) > MAX_INC)
       position_increment_new = position_increment_old;
    */

    // kumulacja przyrostu polozenia w tym makrokroku // ORIGINAL
    // pos_increment_new_sum += position_increment_new*POSTUMENT_TO_TRACK_RATIO;
    // servo_pos_increment_new_sum += position_increment_new*POSTUMENT_TO_TRACK_RATIO; // by Y

    // kumulacja przyrostu polozenia w tym makrokroku
    //pos_increment_new_sum += root_position_increment_new;
    // servo_pos_increment_new_sum += root_position_increment_new;// by Y

    // Przyrost calki uchybu
    delta_eint = delta_eint_old + 1.008*(step_new_pulse - position_increment_new) -
                 0.992*(step_old_pulse - position_increment_old);

    // Sprawdzenie czy numer algorytmu lub zestawu parametrow sie zmienil?
    // Jezeli tak, to nalezy dokonac uaktualnienia numerow (ewentualnie wykryc niewlasciwosc numerow)
    if ( (current_algorithm_no != algorithm_no) ||
            (current_algorithm_parameters_no != algorithm_parameters_no) )
    {
        switch (algorithm_no)
        {
        case 0:  // algorytm nr 0
            switch (algorithm_parameters_no)
            {
            case 0:  // zestaw parametrow nr 0
                current_algorithm_parameters_no = algorithm_parameters_no;
                current_algorithm_no = algorithm_no;
                a  = 0.3079;
                b0 = 1.0942*1.0;
                b1 = 0.9166*1.0;
                k_feedforward = 0.35;
                break;
            case 1: // zestaw parametrow nr 1
                current_algorithm_parameters_no = algorithm_parameters_no;
                current_algorithm_no = algorithm_no;
                a  = 0.3079;
                b0 = 1.0942*2.5;
                b1 = 0.9166*2.5;
                k_feedforward = 0;
                break;
            default: // blad => przywrocic stary algorytm i j stary zestaw parametrow
                algorithm_no = current_algorithm_no;
                algorithm_parameters_no = current_algorithm_parameters_no;
                alg_par_status = UNIDENTIFIED_ALGORITHM_PARAMETERS_NO;
                break;
            }
            break;
        case 1: // algorytm nr 1
            switch (algorithm_parameters_no)
            {
            case 0: // zestaw parametrow nr 0
                current_algorithm_parameters_no = algorithm_parameters_no;
                current_algorithm_no = algorithm_no;
                a  = 0;
                b0 = 0;
                b1 = 0;
                k_feedforward = 0;
                break;
            case 1: // zestaw parametrow nr 1
                current_algorithm_parameters_no = algorithm_parameters_no;
                current_algorithm_no = algorithm_no;
                a  = 0;
                b0 = 0;
                b1 = 0;
                k_feedforward = 0;
                break;
            default: // blad - nie ma takiego zestawu parametrow dla tego algorytmu
                // => przywrocic stary algorytm i j stary zestaw parametrow
                algorithm_no = current_algorithm_no;
                algorithm_parameters_no = current_algorithm_parameters_no;
                alg_par_status = UNIDENTIFIED_ALGORITHM_PARAMETERS_NO;
                break;
            }
            break;
        case 2:  // algorytm nr 2 - sterowanie pradowe
            current_algorithm_parameters_no = algorithm_parameters_no;
            current_algorithm_no = algorithm_no;
            break;
        default: // blad - nie ma takiego algorytmu
            // => przywrocic stary algorytm i j stary zestaw parametrow
            algorithm_no = current_algorithm_no;
            algorithm_parameters_no = current_algorithm_parameters_no;
            alg_par_status = UNIDENTIFIED_ALGORITHM_NO;
            break;
        }
    }

    a=0.315789473684;
    b0=1.997464;
    b1=1.904138;

    switch (algorithm_no)
    {
    case 0:  // algorytm nr 0
        // obliczenie nowej wartosci wypelnienia PWM algorytm PD + I
        set_value_new = (1+a)*set_value_old - a*set_value_very_old +
                        b0*delta_eint - b1*delta_eint_old;
        break;
    case 1:  // algorytm nr 1
        // obliczenie nowej wartosci wypelnienia PWM algorytm PD + I
        set_value_new = (1+a)*set_value_old - a*set_value_very_old +
                        b0*(step_new_pulse - position_increment_new)
                        - b1*(step_old_pulse - position_increment_old);
        break;
    case 2:  // algorytm nr 2 - sterowanie pradowe
        // DUNG START
        current_desired = 0.4*9.52 * (master.new_instruction.arm.pf_def.desired_torque[2] / 158);
        current_measured = (meassured_current - 128 - 3)*0.035;
        current_error = current_desired - current_measured;
        int_current_error =  int_current_error + current_error*0.02;		// 500Hz => 0.02s
        //int_current_error = 0;
        set_value_new = 32 * current_error + 5.5*int_current_error;

        display++;
        if (display >= 500)
        {
            display = 0;
            //printf("joint 3:   current_error = %f,    out = %f\n", current_error, 30 * current_error + 5.0*int_current_error);
        }
        // DUNG END
        break;
    default: // w tym miejscu nie powinien wystapic blad zwiazany z
        // nieistniejacym numerem algorytmu
        set_value_new = 0; // zerowe nowe sterowanie
        break;
    }


    master.rb_obj->lock_mutex();

    master.rb_obj->step_data.desired_inc[3] = (float) step_new_pulse; // pozycja osi 0
    master.rb_obj->step_data.current_inc[3] = (short int) position_increment_new;
    master.rb_obj->step_data.pwm[3] = (float) set_value_new;
    master.rb_obj->step_data.uchyb[3]=(float) (step_new_pulse - position_increment_new);
    // master.rb_obj->step_data.uchyb[3]=(float) (step_new_pulse - position_increment_new);

    master.rb_obj->unlock_mutex();

    // ograniczenie na sterowanie
    if (set_value_new > MAX_PWM)
        set_value_new = MAX_PWM;
    if (set_value_new < -MAX_PWM)
        set_value_new = -MAX_PWM;

    // ograniczenie przyrostu PWM
    // ma na celu zapobiegac osiaganiu zbyt duzych pradow we wzmacniaczach mocy
    if (set_value_new - set_value_old > IRP6_POSTUMENT_AXE4_MAX_PWM_INCREMENT)
        set_value_new = set_value_old + IRP6_POSTUMENT_AXE4_MAX_PWM_INCREMENT;
    if (set_value_new- set_value_old < -IRP6_POSTUMENT_AXE4_MAX_PWM_INCREMENT)
        set_value_new = set_value_old - IRP6_POSTUMENT_AXE4_MAX_PWM_INCREMENT;

    // przepisanie nowych wartosci zmiennych do zmiennych przechowujacych wartosci poprzednie
    position_increment_old = position_increment_new;
    delta_eint_old = delta_eint;
    step_old_pulse = step_new_pulse;
    set_value_very_old = set_value_old;
    set_value_old = set_value_new;
    PWM_value = (int) set_value_new;

    return alg_par_status;

}
/*-----------------------------------------------------------------------*/


/*-----------------------------------------------------------------------*/
uint8_t NL_regulator_5_irp6p::compute_set_value (void)
{
    // algorytm regulacji dla serwomechanizmu

    // position_increment_old - przedostatnio odczytany przyrost polozenie
    //                         (delta y[k-2] -- mierzone w impulsach)
    // position_increment_new - ostatnio odczytany przyrost polozenie
    //                         (delta y[k-1] -- mierzone w impulsach)
    // step_old_pulse               - poprzednia wartosc zadana dla jednego kroku
    //                         regulacji (przyrost wartosci zadanej polozenia --
    //                         delta r[k-2] -- mierzone w impulsach)
    // step_new               - nastepna wartosc zadana dla jednego kroku
    //                         regulacji (przyrost wartosci zadanej polozenia --
    //                         delta r[k-1] -- mierzone w radianach)
    // set_value_new          - wielkosc kroku do realizacji przez HIP
    //                         (wypelnienie PWM -- u[k]): czas trwania jedynki
    // set_value_old          - wielkosc kroku do realizacji przez HIP
    //                         (wypelnienie PWM -- u[k-1]): czas trwania jedynki
    // set_value_very_old     - wielkosc kroku do realizacji przez HIP
    //                         (wypelnienie PWM -- u[k-2]): czas trwania jedynki

    double step_new_pulse; // nastepna wartosc zadana dla jednego kroku regulacji
    // (przyrost wartosci zadanej polozenia --
    // delta r[k-1] -- mierzone w impulsach)
    uint8_t alg_par_status;   // okresla prawidlowosc numeru algorytmu regulacji
    // i zestawu jego parametrow
    //
    double current_error;
    double current_desired;
    double current_measured;
    //


    alg_par_status = ALGORITHM_AND_PARAMETERS_OK;

    // double root_position_increment_new=position_increment_new;

    // BY Y i S - uwzglednie ograniczen na predkosc i przyspieszenie
    constraint_detector(common::SG_REG_5_MAX_ACC, common::SG_REG_5_MAX_SPEED);

    // przeliczenie radianow na impulsy
    // step_new_pulse = step_new*IRP6_POSTUMENT_INC_PER_REVOLUTION/(2*M_PI); // ORIGINAL
    step_new_pulse = step_new*IRP6_POSTUMENT_AXIS_0_TO_5_INC_PER_REVOLUTION/(2*M_PI);
    //position_increment_new= position_increment_new/AXE_0_TO_5_POSTUMENT_TO_TRACK_RATIO;
    /*
    if (!eee)
     if ( (fabs(step_new_pulse) < 0.0001) && (fabs(position_increment_new - position_increment_old ) > 40) ) {
      eee++;
     }
     */
    // if (eee > 0 && eee < 10 ) {
    //  cprintf("O5: svn=%4.0lf svo=%4.0lf svvo=%4.0lf de=%4.0lf deo=%4.0lf snp=%4.0lf pin=%4.0lf pio=%4.0lf\n",set_value_new,set_value_old,
    //  set_value_very_old, delta_eint, delta_eint_old, step_new_pulse,position_increment_new,position_increment_old);
    //  cprintf("O5: snp=%4.0lf pin=%4.0lf pio=%4.0lf L=%4x U=%4x\n", step_new_pulse,position_increment_new,position_increment_old,md.robot_status[4].adr_224,md.robot_status[4].adr_226);
    //  eee++;
    //  if (eee == 9) eee=0;
    // }


    /* // by Y - bez sensu
    // Jesli rzeczywisty przyrost jest wiekszy od dopuszczalnego
     if (fabs(position_increment_new) > MAX_INC)
       position_increment_new = position_increment_old;
    */

    // kumulacja przyrostu polozenia w tym makrokroku // ORIGINAL
    // pos_increment_new_sum += position_increment_new*POSTUMENT_TO_TRACK_RATIO;
    // servo_pos_increment_new_sum += position_increment_new*POSTUMENT_TO_TRACK_RATIO; // by Y

    // kumulacja przyrostu polozenia w tym makrokroku
    // pos_increment_new_sum += root_position_increment_new;
    // servo_pos_increment_new_sum += root_position_increment_new;// by Y

    // Przyrost calki uchybu
    delta_eint = delta_eint_old + 1.010*(step_new_pulse - position_increment_new) -
                 0.990*(step_old_pulse - position_increment_old);

    // Sprawdzenie czy numer algorytmu lub zestawu parametrow sie zmienil?
    // Jezeli tak, to nalezy dokonac uaktualnienia numerow (ewentualnie wykryc niewlasciwosc numerow)
    if ( (current_algorithm_no != algorithm_no) ||
            (current_algorithm_parameters_no != algorithm_parameters_no) )
    {
        switch (algorithm_no)
        {
        case 0:  // algorytm nr 0
            switch (algorithm_parameters_no)
            {
            case 0:  // zestaw parametrow nr 0
                current_algorithm_parameters_no = algorithm_parameters_no;
                current_algorithm_no = algorithm_no;
                a  = 0.3079;
                b0 = 1.0942*1.5;
                b1 = 0.9166*1.5;
                k_feedforward = 0.35;
                break;
            case 1: // zestaw parametrow nr 1
                current_algorithm_parameters_no = algorithm_parameters_no;
                current_algorithm_no = algorithm_no;
                a  = 0.3079;
                b0 = 1.0942*2.5;
                b1 = 0.9166*2.5;
                k_feedforward = 0;
                break;
            default: // blad => przywrocic stary algorytm i j stary zestaw parametrow
                algorithm_no = current_algorithm_no;
                algorithm_parameters_no = current_algorithm_parameters_no;
                alg_par_status = UNIDENTIFIED_ALGORITHM_PARAMETERS_NO;
                break;
            }
            break;
        case 1: // algorytm nr 1
            switch (algorithm_parameters_no)
            {
            case 0: // zestaw parametrow nr 0
                current_algorithm_parameters_no = algorithm_parameters_no;
                current_algorithm_no = algorithm_no;
                a =  0;
                b0 = 0;
                b1 = 0;
                k_feedforward = 0;
                break;
            case 1: // zestaw parametrow nr 1
                current_algorithm_parameters_no = algorithm_parameters_no;
                current_algorithm_no = algorithm_no;
                a = 0;
                b0 = 0;
                b1 = 0;
                k_feedforward = 0;
                break;
            default: // blad - nie ma takiego zestawu parametrow dla tego algorytmu
                // => przywrocic stary algorytm i j stary zestaw parametrow
                algorithm_no = current_algorithm_no;
                algorithm_parameters_no = current_algorithm_parameters_no;
                alg_par_status = UNIDENTIFIED_ALGORITHM_PARAMETERS_NO;
                break;
            }
            break;
        case 2:  // algorytm nr 2 - sterowanie pradowe
            current_algorithm_parameters_no = algorithm_parameters_no;
            current_algorithm_no = algorithm_no;
            break;
        default: // blad - nie ma takiego algorytmu
            // => przywrocic stary algorytm i j stary zestaw parametrow
            algorithm_no = current_algorithm_no;
            algorithm_parameters_no = current_algorithm_parameters_no;
            alg_par_status = UNIDENTIFIED_ALGORITHM_NO;
            break;
        }
    }


    a=0.548946716233;
    b0=1.576266; //9.244959545156;
    b1=1.468599; //8.613484947882;


    switch (algorithm_no)
    {
    case 0:  // algorytm nr 0
        // obliczenie nowej wartosci wypelnienia PWM algorytm PD + I
        set_value_new = (1+a)*set_value_old - a*set_value_very_old +
                        b0*delta_eint - b1*delta_eint_old;
        break;
    case 1:  // algorytm nr 1
        // obliczenie nowej wartosci wypelnienia PWM algorytm PD + I
        set_value_new = (1+a)*set_value_old - a*set_value_very_old +
                        b0*(step_new_pulse - position_increment_new)
                        - b1*(step_old_pulse - position_increment_old);
        break;
    case 2:  // algorytm nr 2 - sterowanie pradowe
        // DUNG START
        current_desired = 0.4*9.52 * (master.new_instruction.arm.pf_def.desired_torque[3] / 158);
        current_measured = (meassured_current - 128 - 3)*0.035;
        current_error = current_desired - current_measured;
        int_current_error =  int_current_error + current_error*0.02;		// 500Hz => 0.02s
        //int_current_error = 0;
        set_value_new = -31 * current_error - 5.3*int_current_error;

        display++;
        if (display >= 500)
        {
            display = 0;
            printf("joint 4:   desired_current = %f,    current_error = %f,    out = %f\n", master.new_instruction.arm.pf_def.desired_torque[3] / 158, current_error, -30 * current_error - 4.0*int_current_error);
        }
        // DUNG END
        break;
    default: // w tym miejscu nie powinien wystapic blad zwiazany z
        // nieistniejacym numerem algorytmu
        set_value_new = 0; // zerowe nowe sterowanie
        break;
    }

    master.rb_obj->lock_mutex();

    master.rb_obj->step_data.desired_inc[4] = (float) step_new_pulse; // pozycja osi 0
    master.rb_obj->step_data.current_inc[4] = (short int) position_increment_new;
    master.rb_obj->step_data.pwm[4] = (float) set_value_new;
    master.rb_obj->step_data.uchyb[4]=(float) (step_new_pulse - position_increment_new);

    master.rb_obj->unlock_mutex();

    // ograniczenie na sterowanie
    if (set_value_new > MAX_PWM)
        set_value_new = MAX_PWM;
    if (set_value_new < -MAX_PWM)
        set_value_new = -MAX_PWM;

    // ograniczenie przyrostu PWM
    // ma na celu zapobiegac osiaganiu zbyt duzych pradow we wzmacniaczach mocy
    if (set_value_new - set_value_old > IRP6_POSTUMENT_AXE5_MAX_PWM_INCREMENT)
        set_value_new = set_value_old + IRP6_POSTUMENT_AXE5_MAX_PWM_INCREMENT;
    if (set_value_new- set_value_old < -IRP6_POSTUMENT_AXE5_MAX_PWM_INCREMENT)
        set_value_new = set_value_old - IRP6_POSTUMENT_AXE5_MAX_PWM_INCREMENT;

    // if (fabs(set_value_new) > 200.0 && first) {
    // cprintf("PIN=%lf PIO=%lf DIN=%lf DIO=%lf SO=%lf SVVO=%lf SV0=%lf\n", position_increment_new,
    //     position_increment_old, delta_eint, delta_eint_old,
    //     step_old_pulse, set_value_very_old, set_value_old);
    // first = false;
    // }


    // przepisanie nowych wartosci zmiennych do zmiennych przechowujacych wartosci poprzednie
    position_increment_old = position_increment_new;
    delta_eint_old = delta_eint;
    step_old_pulse = step_new_pulse;
    set_value_very_old = set_value_old;
    set_value_old = set_value_new;
    PWM_value = (int) set_value_new;

    return alg_par_status;

}
/*-----------------------------------------------------------------------*/


/*-----------------------------------------------------------------------*/
uint8_t NL_regulator_6_irp6p::compute_set_value (void)
{
    // algorytm regulacji dla serwomechanizmu

    // position_increment_old - przedostatnio odczytany przyrost polozenie
    //                         (delta y[k-2] -- mierzone w impulsach)
    // position_increment_new - ostatnio odczytany przyrost polozenie
    //                         (delta y[k-1] -- mierzone w impulsach)
    // step_old_pulse               - poprzednia wartosc zadana dla jednego kroku
    //                         regulacji (przyrost wartosci zadanej polozenia --
    //                         delta r[k-2] -- mierzone w impulsach)
    // step_new               - nastepna wartosc zadana dla jednego kroku
    //                         regulacji (przyrost wartosci zadanej polozenia --
    //                         delta r[k-1] -- mierzone w radianach)
    // set_value_new          - wielkosc kroku do realizacji przez HIP
    //                         (wypelnienie PWM -- u[k]): czas trwania jedynki
    // set_value_old          - wielkosc kroku do realizacji przez HIP
    //                         (wypelnienie PWM -- u[k-1]): czas trwania jedynki
    // set_value_very_old     - wielkosc kroku do realizacji przez HIP
    //                         (wypelnienie PWM -- u[k-2]): czas trwania jedynki

    double step_new_pulse; // nastepna wartosc zadana dla jednego kroku regulacji
    // (przyrost wartosci zadanej polozenia --
    // delta r[k-1] -- mierzone w impulsach)
    uint8_t alg_par_status;   // okresla prawidlowosc numeru algorytmu regulacji
    // i zestawu jego parametrow
    //
    double current_error;
    double current_desired;
    double current_measured;
    //

    alg_par_status = ALGORITHM_AND_PARAMETERS_OK;

    double root_position_increment_new=position_increment_new;

    // BY Y i S - uwzglednie ograniczen na predkosc i przyspieszenie
    constraint_detector(common::SG_REG_6_MAX_ACC, common::SG_REG_6_MAX_SPEED);

    // przeliczenie radianow na impulsy
    // step_new_pulse = step_new*IRP6_POSTUMENT_INC_PER_REVOLUTION/(2*M_PI); // ORIGINAL
    step_new_pulse = step_new*IRP6_POSTUMENT_AXIS_0_TO_5_INC_PER_REVOLUTION/(2*M_PI);
    //position_increment_new= position_increment_new/AXE_0_TO_5_POSTUMENT_TO_TRACK_RATIO;

    /*
    if (!fff)
     if ( (fabs(step_new_pulse) < 0.0001) && (fabs(position_increment_new - position_increment_old ) > 40) ) {
      fff++;
     }
     */
    // if (fff > 0 && fff < 10 ) {
    //  cprintf("O6: svn=%4.0lf svo=%4.0lf svvo=%4.0lf de=%4.0lf deo=%4.0lf snp=%4.0lf pin=%4.0lf pio=%4.0lf\n",set_value_new,set_value_old,
    //  set_value_very_old, delta_eint, delta_eint_old, step_new_pulse,position_increment_new,position_increment_old);
    //  cprintf("O6: snp=%4.0lf pin=%4.0lf pio=%4.0lf L=%4x U=%4x\n", step_new_pulse,position_increment_new,position_increment_old,md.robot_status[5].adr_224,md.robot_status[5].adr_226);
    //  fff++;
    //  if (fff == 9) fff=0;
    // }


    /* // by Y - bez sensu
    // Jesli rzeczywisty przyrost jest wiekszy od dopuszczalnego
     if (fabs(position_increment_new) > MAX_INC)
       position_increment_new = position_increment_old;
    */

    // kumulacja przyrostu polozenia w tym makrokroku // ORIGINAL
    // pos_increment_new_sum += position_increment_new*POSTUMENT_TO_TRACK_RATIO;
    // servo_pos_increment_new_sum += position_increment_new*POSTUMENT_TO_TRACK_RATIO; // by Y

    // kumulacja przyrostu polozenia w tym makrokroku
    pos_increment_new_sum += root_position_increment_new;
    servo_pos_increment_new_sum += root_position_increment_new;// by Y

    // Przyrost calki uchybu
    delta_eint = delta_eint_old + 1.020*(step_new_pulse - position_increment_new) -
                 0.980*(step_old_pulse - position_increment_old);

    // Sprawdzenie czy numer algorytmu lub zestawu parametrow sie zmienil?
    // Jezeli tak, to nalezy dokonac uaktualnienia numerow (ewentualnie wykryc niewlasciwosc numerow)
    if ( (current_algorithm_no != algorithm_no) ||
            (current_algorithm_parameters_no != algorithm_parameters_no) )
    {
        switch (algorithm_no)
        {
        case 0:  // algorytm nr 0
            switch (algorithm_parameters_no)
            {
            case 0:  // zestaw parametrow nr 0
                current_algorithm_parameters_no = algorithm_parameters_no;
                current_algorithm_no = algorithm_no;
                a = 0.3079;
                b0 = 1.0942*1.5;
                b1 = 0.9166*1.5;
                k_feedforward = 0.35;
                break;
            case 1: // zestaw parametrow nr 1
                current_algorithm_parameters_no = algorithm_parameters_no;
                current_algorithm_no = algorithm_no;
                a  = 0.3079;
                b0 = 1.0942*2.5;
                b1 = 0.9166*2.5;
                k_feedforward = 0;
                break;
            default: // blad => przywrocic stary algorytm i j stary zestaw parametrow
                algorithm_no = current_algorithm_no;
                algorithm_parameters_no = current_algorithm_parameters_no;
                alg_par_status = UNIDENTIFIED_ALGORITHM_PARAMETERS_NO;
                break;
            }
            break;
        case 1: // algorytm nr 1
            switch (algorithm_parameters_no)
            {
            case 0: // zestaw parametrow nr 0
                current_algorithm_parameters_no = algorithm_parameters_no;
                current_algorithm_no = algorithm_no;
                a  = 0;
                b0 = 0;
                b1 = 0;
                k_feedforward = 0;
                break;
            case 1: // zestaw parametrow nr 1
                current_algorithm_parameters_no = algorithm_parameters_no;
                current_algorithm_no = algorithm_no;
                a  = 0;
                b0 = 0;
                b1 = 0;
                k_feedforward = 0;
                break;
            default: // blad - nie ma takiego zestawu parametrow dla tego algorytmu
                // => przywrocic stary algorytm i j stary zestaw parametrow
                algorithm_no = current_algorithm_no;
                algorithm_parameters_no = current_algorithm_parameters_no;
                alg_par_status = UNIDENTIFIED_ALGORITHM_PARAMETERS_NO;
                break;
            }
            break;
        case 2:  // algorytm nr 2 - sterowanie pradowe
            current_algorithm_parameters_no = algorithm_parameters_no;
            current_algorithm_no = algorithm_no;
            break;
        default: // blad - nie ma takiego algorytmu
            // => przywrocic stary algorytm i j stary zestaw parametrow
            algorithm_no = current_algorithm_no;
            algorithm_parameters_no = current_algorithm_parameters_no;
            alg_par_status = UNIDENTIFIED_ALGORITHM_NO;
            break;
        }
    }

    a=0.391982182628;
    b0=1.114648; //6.537527839644;
    b1=1.021348; //5.990311804009;


    switch (algorithm_no)
    {
    case 0:  // algorytm nr 0
        // obliczenie nowej wartosci wypelnienia PWM algorytm PD + I
        set_value_new = (1+a)*set_value_old - a*set_value_very_old +
                        b0*delta_eint - b1*delta_eint_old;
        break;
    case 1:  // algorytm nr 1
        // obliczenie nowej wartosci wypelnienia PWM algorytm PD + I
        set_value_new = (1+a)*set_value_old - a*set_value_very_old +
                        b0*(step_new_pulse - position_increment_new)
                        - b1*(step_old_pulse - position_increment_old);
        break;
    case 2:  // algorytm nr 2 - sterowanie pradowe
        // DUNG START
        current_desired = 0.4*9.52 * (master.new_instruction.arm.pf_def.desired_torque[4] / 158);
        current_measured = (meassured_current - 128 - 3)*0.035;
        current_error = current_desired - current_measured;
        int_current_error =  int_current_error + current_error*0.02;		// 500Hz => 0.02s
        //int_current_error = 0;
        set_value_new = -33 * current_error - 6.0*int_current_error;

        display++;
        if (display >= 500)
        {
            display = 0;
            //printf("joint 5:   desired_current = %f,    current_error = %f,    out = %f\n", master.new_instruction.arm.pf_def.desired_torque[4] / 158, current_error, -30 * current_error - 4.0*int_current_error);
        }
        // DUNG END
        break;
    default: // w tym miejscu nie powinien wystapic blad zwiazany z
        // nieistniejacym numerem algorytmu
        set_value_new = 0; // zerowe nowe sterowanie
        break;
    }

    // ograniczenie na sterowanie
    if (set_value_new > MAX_PWM)
        set_value_new = MAX_PWM;
    if (set_value_new < -MAX_PWM)
        set_value_new = -MAX_PWM;

    // ograniczenie przyrostu PWM
    // ma na celu zapobiegac osiaganiu zbyt duzych pradow we wzmacniaczach mocy
    if (set_value_new - set_value_old > IRP6_POSTUMENT_AXE6_MAX_PWM_INCREMENT)
        set_value_new = set_value_old + IRP6_POSTUMENT_AXE6_MAX_PWM_INCREMENT;
    if (set_value_new- set_value_old < -IRP6_POSTUMENT_AXE6_MAX_PWM_INCREMENT)
        set_value_new = set_value_old - IRP6_POSTUMENT_AXE6_MAX_PWM_INCREMENT;

    master.rb_obj->lock_mutex();

    master.rb_obj->step_data.desired_inc[5] = (float) step_new_pulse; // pozycja osi 0
    master.rb_obj->step_data.current_inc[5] = (short int) position_increment_new;
    master.rb_obj->step_data.pwm[5] = (float) set_value_new;
    master.rb_obj->step_data.uchyb[5]=(float) (step_new_pulse - position_increment_new);

    master.rb_obj->unlock_mutex();

    // if (set_value_new > 0.0) {
    //  cprintf("svn = %lf  pin = %lf\n",set_value_new, position_increment_new);
    // }

    // przepisanie nowych wartosci zmiennych do zmiennych przechowujacych wartosci poprzednie
    position_increment_old = position_increment_new;
    delta_eint_old = delta_eint;
    step_old_pulse = step_new_pulse;
    set_value_very_old = set_value_old;
    set_value_old = set_value_new;
    PWM_value = (int) set_value_new;

    return alg_par_status;

}
/*-----------------------------------------------------------------------*/


/*-----------------------------------------------------------------------*/
uint8_t NL_regulator_7_irp6p::compute_set_value (void)
{
    // algorytm regulacji dla serwomechanizmu

    // position_increment_old - przedostatnio odczytany przyrost polozenia
    //                         (delta y[k-2] -- mierzone w impulsach)
    // position_increment_new - ostatnio odczytany przyrost polozenia
    //                         (delta y[k-1] -- mierzone w impulsach)
    // step_old_pulse               - poprzednia wartosc zadana dla jednego kroku
    //                         regulacji (przyrost wartosci zadanej polozenia --
    //                         delta r[k-2] -- mierzone w impulsach)
    // step_new               - nastepna wartosc zadana dla jednego kroku
    //                         regulacji (przyrost wartosci zadanej polozenia --
    //                         delta r[k-1] -- mierzone w radianach)
    // set_value_new          - wielkosc kroku do realizacji przez HIP
    //                         (wypelnienie PWM -- u[k]): czas trwania jedynki
    // set_value_old          - wielkosc kroku do realizacji przez HIP
    //                         (wypelnienie PWM -- u[k-1]): czas trwania jedynki
    // set_value_very_old     - wielkosc kroku do realizacji przez HIP
    //                         (wypelnienie PWM -- u[k-2]): czas trwania jedynki

    double step_new_pulse; // nastepna wartosc zadana dla jednego kroku regulacji
    // (przyrost wartosci zadanej polozenia --
    // delta r[k-1] -- mierzone w impulsach)
    uint8_t alg_par_status;   // okresla prawidlowosc numeru algorytmu regulacji
    // i zestawu jego parametrow
    //
    double current_error;
    double current_desired;
    double current_measured;
    //

    alg_par_status = ALGORITHM_AND_PARAMETERS_OK;

    // double root_position_increment_new=position_increment_new;

    // BY Y i S - uwzglednie ograniczen na predkosc i przyspieszenie
    constraint_detector(common::SG_REG_7_MAX_ACC, common::SG_REG_7_MAX_SPEED);

    // przeliczenie radianow na impulsy
    // step_new_pulse = step_new*IRP6_POSTUMENT_AXIS_6_INC_PER_REVOLUTION/(2*M_PI); // ORIGINAL
    step_new_pulse = step_new*IRP6_POSTUMENT_AXIS_6_INC_PER_REVOLUTION/(2*M_PI);
    //position_increment_new= position_increment_new/AXE_6_POSTUMENT_TO_TRACK_RATIO;

    // if (step_new!=0.0) printf(" 7 reg:%f\n", step_new);

    /*
    if (!fff)
     if ( (fabs(step_new_pulse) < 0.0001) && (fabs(position_increment_new - position_increment_old ) > 40) ) {
      fff++;
     }
     */
    // if (fff > 0 && fff < 10 ) {
    //  cprintf("O6: svn=%4.0lf svo=%4.0lf svvo=%4.0lf de=%4.0lf deo=%4.0lf snp=%4.0lf pin=%4.0lf pio=%4.0lf\n",set_value_new,set_value_old,
    //  set_value_very_old, delta_eint, delta_eint_old, step_new_pulse,position_increment_new,position_increment_old);
    //  cprintf("O6: snp=%4.0lf pin=%4.0lf pio=%4.0lf L=%4x U=%4x\n", step_new_pulse,position_increment_new,position_increment_old,md.robot_status[5].adr_224,md.robot_status[5].adr_226);
    //  fff++;
    //  if (fff == 9) fff=0;
    // }


    /* // by Y - bez sensu
    // Jesli rzeczywisty przyrost jest wiekszy od dopuszczalnego
     if (fabs(position_increment_new) > MAX_INC)
       position_increment_new = position_increment_old;
    */

    // kumulacja przyrostu polozenia w tym makrokroku // ORIGINAL
    // pos_increment_new_sum += position_increment_new*POSTUMENT_TO_TRACK_RATIO;
    // servo_pos_increment_new_sum += position_increment_new*POSTUMENT_TO_TRACK_RATIO; // by Y

    // kumulacja przyrostu polozenia w tym makrokroku
    //pos_increment_new_sum += root_position_increment_new;
    //servo_pos_increment_new_sum += root_position_increment_new;// by Y

    // Przyrost calki uchybu
    delta_eint = delta_eint_old + 1.020*(step_new_pulse - position_increment_new) -
                 0.980*(step_old_pulse - position_increment_old);

    // Sprawdzenie czy numer algorytmu lub zestawu parametrow sie zmienil?
    // Jezeli tak, to nalezy dokonac uaktualnienia numerow (ewentualnie wykryc niewlasciwosc numerow)
    if ( (current_algorithm_no != algorithm_no) ||
            (current_algorithm_parameters_no != algorithm_parameters_no) )
    {
        switch (algorithm_no)
        {
        case 0:  // algorytm nr 0
            switch (algorithm_parameters_no)
            {
            case 0:  // zestaw parametrow nr 0
                current_algorithm_parameters_no = algorithm_parameters_no;
                current_algorithm_no = algorithm_no;
                a = 0.3079;
                b0 = 1.0942*1.5;
                b1 = 0.9166*1.5;
                k_feedforward = 0.35;
                break;
            case 1: // zestaw parametrow nr 1
                current_algorithm_parameters_no = algorithm_parameters_no;
                current_algorithm_no = algorithm_no;
                a  = 0.3079;
                b0 = 1.0942*2.5;
                b1 = 0.9166*2.5;
                k_feedforward = 0;
                break;
            default: // blad => przywrocic stary algorytm i j stary zestaw parametrow
                algorithm_no = current_algorithm_no;
                algorithm_parameters_no = current_algorithm_parameters_no;
                alg_par_status = UNIDENTIFIED_ALGORITHM_PARAMETERS_NO;
                break;
            }
            break;
        case 1: // algorytm nr 1
            switch (algorithm_parameters_no)
            {
            case 0: // zestaw parametrow nr 0
                current_algorithm_parameters_no = algorithm_parameters_no;
                current_algorithm_no = algorithm_no;
                a  = 0;
                b0 = 0;
                b1 = 0;
                k_feedforward = 0;
                break;
            case 1: // zestaw parametrow nr 1
                current_algorithm_parameters_no = algorithm_parameters_no;
                current_algorithm_no = algorithm_no;
                a  = 0;
                b0 = 0;
                b1 = 0;
                k_feedforward = 0;
                break;
            default: // blad - nie ma takiego zestawu parametrow dla tego algorytmu
                // => przywrocic stary algorytm i j stary zestaw parametrow
                algorithm_no = current_algorithm_no;
                algorithm_parameters_no = current_algorithm_parameters_no;
                alg_par_status = UNIDENTIFIED_ALGORITHM_PARAMETERS_NO;
                break;
            }
            ; // end: switch (algorithm_parameters_no)
            break;
        case 2:  // algorytm nr 2 - sterowanie pradowe
            current_algorithm_parameters_no = algorithm_parameters_no;
            current_algorithm_no = algorithm_no;
            break;
        default: // blad - nie ma takiego algorytmu
            // => przywrocic stary algorytm i j stary zestaw parametrow
            algorithm_no = current_algorithm_no;
            algorithm_parameters_no = current_algorithm_parameters_no;
            alg_par_status = UNIDENTIFIED_ALGORITHM_NO;
            break;
        }
    }

    /*
    a=0.391982182628;
    b0=6.537527839644;
    b1=5.990311804009;
    */

    a=0.3;
    b0=1.364; //4
    b1=1.264; //1.364//4

    switch (algorithm_no)
    {
    case 0:  // algorytm nr 0
        // obliczenie nowej wartosci wypelnienia PWM algorytm PD + I
        set_value_new = (1+a)*set_value_old - a*set_value_very_old +
                        b0*delta_eint - b1*delta_eint_old;
        break;
    case 1:  // algorytm nr 1
        // obliczenie nowej wartosci wypelnienia PWM algorytm PD + I
        set_value_new = (1+a)*set_value_old - a*set_value_very_old +
                        b0*(step_new_pulse - position_increment_new)
                        - b1*(step_old_pulse - position_increment_old);
        break;
    case 2:  // algorytm nr 2 - sterowanie pradowe
        // DUNG START
        current_desired = 0.4*9.52 * (master.new_instruction.arm.pf_def.desired_torque[5] / 158);
        current_measured = (meassured_current - 128 - 3)*0.02;
        current_error = current_desired - current_measured;
        int_current_error =  int_current_error + current_error*0.02;		// 500Hz => 0.02s
        //int_current_error = 0;
        set_value_new = -30 * current_error - 4.0*int_current_error;

        display++;
        if (display >= 500)
        {
            display = 0;
            printf("joint 6:   desired_current = %f,    current_error = %f,    out = %f\n", master.new_instruction.arm.pf_def.desired_torque[5] / 158, current_error, -30 * current_error - 4.0*int_current_error);
        }
        // DUNG END
        break;
    default: // w tym miejscu nie powinien wystapic blad zwiazany z
        // nieistniejacym numerem algorytmu
        set_value_new = 0; // zerowe nowe sterowanie
        break;
    }

    // ograniczenie na sterowanie
    if (set_value_new > MAX_PWM)
        set_value_new = MAX_PWM;
    if (set_value_new < -MAX_PWM)
        set_value_new = -MAX_PWM;


    // if (set_value_new!=0.0) printf ("aa: %f\n", set_value_new);

    // ograniczenie przyrostu PWM
    // ma na celu zapobiegac osiaganiu zbyt duzych pradow we wzmacniaczach mocy
    if (set_value_new - set_value_old > IRP6_POSTUMENT_AXE7_MAX_PWM_INCREMENT)
        set_value_new = set_value_old + IRP6_POSTUMENT_AXE7_MAX_PWM_INCREMENT;
    if (set_value_new- set_value_old < -IRP6_POSTUMENT_AXE7_MAX_PWM_INCREMENT)
        set_value_new = set_value_old - IRP6_POSTUMENT_AXE7_MAX_PWM_INCREMENT;

    master.rb_obj->lock_mutex();

    master.rb_obj->step_data.desired_inc[6] = (float) step_new_pulse; // pozycja osi 0
    master.rb_obj->step_data.current_inc[6] = (short int) position_increment_new;
    master.rb_obj->step_data.pwm[6] = (float) set_value_new;
    master.rb_obj->step_data.uchyb[6]=(float) (step_new_pulse - position_increment_new);

    master.rb_obj->unlock_mutex();

    // if (set_value_new > 0.0) {
    //  cprintf("svn = %lf  pin = %lf\n",set_value_new, position_increment_new);
    // }

    // przepisanie nowych wartosci zmiennych do zmiennych przechowujacych wartosci poprzednie
    position_increment_old = position_increment_new;
    delta_eint_old = delta_eint;
    step_old_pulse = step_new_pulse;
    set_value_very_old = set_value_old;
    set_value_old = set_value_new;
    PWM_value = (int) set_value_new;

    return alg_par_status;

}
/*-----------------------------------------------------------------------*/


/*-----------------------------------------------------------------------*/
uint8_t NL_regulator_8_irp6p::compute_set_value (void)
{
    // algorytm regulacji dla serwomechanizmu

    // position_increment_old - przedostatnio odczytany przyrost polozenie
    //                         (delta y[k-2] -- mierzone w impulsach)
    // position_increment_new - ostatnio odczytany przyrost polozenie
    //                         (delta y[k-1] -- mierzone w impulsach)
    // step_old_pulse               - poprzednia wartosc zadana dla jednego kroku
    //                         regulacji (przyrost wartosci zadanej polozenia --
    //                         delta r[k-2] -- mierzone w impulsach)
    // step_new               - nastepna wartosc zadana dla jednego kroku
    //                         regulacji (przyrost wartosci zadanej polozenia --
    //                         delta r[k-1] -- mierzone w radianach)
    // set_value_new          - wielkosc kroku do realizacji przez HIP
    //                         (wypelnienie PWM -- u[k]): czas trwania jedynki
    // set_value_old          - wielkosc kroku do realizacji przez HIP
    //                         (wypelnienie PWM -- u[k-1]): czas trwania jedynki
    // set_value_very_old     - wielkosc kroku do realizacji przez HIP
    //                         (wypelnienie PWM -- u[k-2]): czas trwania jedynki

    double step_new_pulse; // nastepna wartosc zadana dla jednego kroku regulacji
    // (przyrost wartosci zadanej polozenia --
    // delta r[k-1] -- mierzone w impulsach)
    uint8_t alg_par_status;   // okresla prawidlowosc numeru algorytmu regulacji
    // i zestawu jego parametrow

    double current_error;
    double current_desired;
    double current_measured;
    static int low_measure_counter;


    alg_par_status = ALGORITHM_AND_PARAMETERS_OK;

    // double root_position_increment_new=position_increment_new;

    // BY Y i S - uwzglednie ograniczen na predkosc i przyspieszenie
    constraint_detector(common::SG_REG_8_MAX_ACC, common::SG_REG_8_MAX_SPEED);

    // przeliczenie radianow na impulsy
    // step_new_pulse = step_new*IRP6_POSTUMENT_INC_PER_REVOLUTION/(2*M_PI); // ORIGINAL
    step_new_pulse = step_new*IRP6_POSTUMENT_AXIS_7_INC_PER_REVOLUTION/(2*M_PI);//*AXE_7_POSTUMENT_TO_TRACK_RATIO);
    //position_increment_new= position_increment_new/AXE_7_POSTUMENT_TO_TRACK_RATIO;


    // if (step_new!=0.0) printf(" 8 reg:%f\n", step_new);

    /*
    if (!fff)
     if ( (fabs(step_new_pulse) < 0.0001) && (fabs(position_increment_new - position_increment_old ) > 40) ) {
      fff++;
     }
     */
    // if (fff > 0 && fff < 10 ) {
    //  cprintf("O6: svn=%4.0lf svo=%4.0lf svvo=%4.0lf de=%4.0lf deo=%4.0lf snp=%4.0lf pin=%4.0lf pio=%4.0lf\n",set_value_new,set_value_old,
    //  set_value_very_old, delta_eint, delta_eint_old, step_new_pulse,position_increment_new,position_increment_old);
    //  cprintf("O6: snp=%4.0lf pin=%4.0lf pio=%4.0lf L=%4x U=%4x\n", step_new_pulse,position_increment_new,position_increment_old,md.robot_status[5].adr_224,md.robot_status[5].adr_226);
    //  fff++;
    //  if (fff == 9) fff=0;
    // }


    /* // by Y - bez sensu
    // Jesli rzeczywisty przyrost jest wiekszy od dopuszczalnego
     if (fabs(position_increment_new) > MAX_INC)
       position_increment_new = position_increment_old;
    */

    // kumulacja przyrostu polozenia w tym makrokroku // ORIGINAL
    // pos_increment_new_sum += position_increment_new*POSTUMENT_TO_TRACK_RATIO;
    // servo_pos_increment_new_sum += position_increment_new*POSTUMENT_TO_TRACK_RATIO; // by Y

    // kumulacja przyrostu polozenia w tym makrokroku
    //pos_increment_new_sum += root_position_increment_new;
    // servo_pos_increment_new_sum += root_position_increment_new;// by Y

    // Przyrost calki uchybu
    delta_eint = delta_eint_old + 1.020*(step_new_pulse - position_increment_new) -
                 0.980*(step_old_pulse - position_increment_old);

    // Sprawdzenie czy numer algorytmu lub zestawu parametrow sie zmienil?
    // Jezeli tak, to nalezy dokonac uaktualnienia numerow (ewentualnie wykryc niewlasciwosc numerow)
    if ( (current_algorithm_no != algorithm_no) ||
            (current_algorithm_parameters_no != algorithm_parameters_no) )
    {
        switch (algorithm_no)
        {
        case 0:  // algorytm nr 0
            switch (algorithm_parameters_no)
            {
            case 0:  // zestaw parametrow nr 0
                current_algorithm_parameters_no = algorithm_parameters_no;
                current_algorithm_no = algorithm_no;
                a = 0.3079;
                b0 = 1.0942*1.5;
                b1 = 0.9166*1.5;
                k_feedforward = 0.35;
                break;
            case 1: // zestaw parametrow nr 1
                current_algorithm_parameters_no = algorithm_parameters_no;
                current_algorithm_no = algorithm_no;
                a  = 0.3079;
                b0 = 1.0942*2.5;
                b1 = 0.9166*2.5;
                k_feedforward = 0;
                break;
            default: // blad => przywrocic stary algorytm i j stary zestaw parametrow
                algorithm_no = current_algorithm_no;
                algorithm_parameters_no = current_algorithm_parameters_no;
                alg_par_status = UNIDENTIFIED_ALGORITHM_PARAMETERS_NO;
                break;
            }
            ; // end: switch (algorithm_parameters_no)
            break;
        case 1: // algorytm nr 1
            switch (algorithm_parameters_no)
            {
            case 0: // zestaw parametrow nr 0
                current_algorithm_parameters_no = algorithm_parameters_no;
                current_algorithm_no = algorithm_no;
                a  = 0;
                b0 = 0;
                b1 = 0;
                k_feedforward = 0;
                break;
            case 1: // zestaw parametrow nr 1
                current_algorithm_parameters_no = algorithm_parameters_no;
                current_algorithm_no = algorithm_no;
                a  = 0;
                b0 = 0;
                b1 = 0;
                k_feedforward = 0;
                break;
            default: // blad - nie ma takiego zestawu parametrow dla tego algorytmu
                // => przywrocic stary algorytm i j stary zestaw parametrow
                algorithm_no = current_algorithm_no;
                algorithm_parameters_no = current_algorithm_parameters_no;
                alg_par_status = UNIDENTIFIED_ALGORITHM_PARAMETERS_NO;
                break;
            }
            ; // end: switch (algorithm_parameters_no)
            break;
           case 2:  // algorytm nr 2 - sterowanie pradowe
                current_algorithm_parameters_no = algorithm_parameters_no;
                current_algorithm_no = algorithm_no;
                break;
        default: // blad - nie ma takiego algorytmu
            // => przywrocic stary algorytm i j stary zestaw parametrow
            algorithm_no = current_algorithm_no;
            algorithm_parameters_no = current_algorithm_parameters_no;
            alg_par_status = UNIDENTIFIED_ALGORITHM_NO;
            break;
        }
    }
    /*
    a=0.391982182628;
    b0=6.537527839644;
    b1=5.990311804009;
    */

    a=0.2; //0.3
    b0=15.984375; //15.984375; //3
    b1=15.784375; //15.984375; //3
    //14.4
    //a=0.2;
    //b0=15.984375;
    //b1=15.984375;

#define PROP_I_REG 0.0
#define INT_I_REG 0.4
#define MAX_REG_CURRENT 15.0

    switch (algorithm_no)
    {
    case 0:  // algorytm nr 0
        //	if (meassured_current != 0) fprintf(stdout,"alg 0: %d\n", meassured_current);

            set_value_new = (1+a)*set_value_old - a*set_value_very_old +
                            b0*delta_eint - b1*delta_eint_old;

            if (set_value_new > MAX_PWM)
                set_value_new = MAX_PWM;
            if (set_value_new < -MAX_PWM)
                set_value_new = -MAX_PWM;

            if (set_value_new - set_value_old > IRP6_POSTUMENT_AXE8_MAX_PWM_INCREMENT)
                set_value_new = set_value_old + IRP6_POSTUMENT_AXE8_MAX_PWM_INCREMENT;
            if (set_value_new- set_value_old < -IRP6_POSTUMENT_AXE8_MAX_PWM_INCREMENT)
                set_value_new = set_value_old - IRP6_POSTUMENT_AXE8_MAX_PWM_INCREMENT;


            set_value_old = set_value_new;


            // wyznaczenie wartosci zadanej pradu
            current_desired = (MAX_REG_CURRENT * set_value_new)/MAX_PWM;

            // ustalenie znaku pradu zmierzonego na podstawie znaku pwm
			  if (set_value_new>0) current_measured = (float)meassured_current;
				else current_measured = (float) (-meassured_current);


			  // wyznaczenie uchybu
			  current_error = current_desired - current_measured;

			  // wyznaczenie calki uchybu
			  int_current_error =  int_current_error + INT_I_REG * current_error;		// 500Hz => 0.02s

			  // przycinanie calki uchybu

			  if (int_current_error >  MAX_PWM) int_current_error =  MAX_PWM;
			  if (int_current_error < -MAX_PWM) int_current_error = -MAX_PWM;

			  if (current_desired>=1)
			  {
				  low_measure_counter=0;
			 // 	if (int_current_error<0) int_current_error = 0;
			  }
			  else if((current_desired<1) && (current_desired>-1))
			  {
				if((++low_measure_counter) >=10)
				{
				int_current_error = 0;
				}
			  }
			  else if (current_desired<=-1)
			  {
				  low_measure_counter=0;
			  //	if (int_current_error>0) int_current_error = 0;
			  }


			  // wyznaczenie nowego sterowania
			  set_value_new = PROP_I_REG * current_error + int_current_error;
/*
             display++;
             if ((display % 1)==0)
             {
               //  display = 0;
                 printf("joint 7:  current_desired = %f,  meassured_current = %f, int_current_error = %f,  set_value_new = %f \n",
                		 current_desired,   current_measured, int_current_error, set_value_new);
             }

*/

            break;

    case 1:  // algorytm nr 1
        //	if (meassured_current != 0) fprintf(stdout,"alg 0: %d\n", meassured_current);
            // obliczenie nowej wartosci wypelnienia PWM algorytm PD + I
            set_value_new = (1+a)*set_value_old - a*set_value_very_old +
                            b0*delta_eint - b1*delta_eint_old;

            if (set_value_new > MAX_PWM)
                set_value_new = MAX_PWM;
            if (set_value_new < -MAX_PWM)
                set_value_new = -MAX_PWM;

            if (set_value_new - set_value_old > IRP6_POSTUMENT_AXE8_MAX_PWM_INCREMENT)
                set_value_new = set_value_old + IRP6_POSTUMENT_AXE8_MAX_PWM_INCREMENT;
            if (set_value_new- set_value_old < -IRP6_POSTUMENT_AXE8_MAX_PWM_INCREMENT)
                set_value_new = set_value_old - IRP6_POSTUMENT_AXE8_MAX_PWM_INCREMENT;


            set_value_old = set_value_new;

            break;
    case 2:  // algorytm nr 2 - sterowanie pradowe
        // DUNG START

    	// ustalenie wartosci zadanej
        if (display<1000) current_desired = 15;
        else if (display<2000) current_desired = -15;
        else current_desired = 0;

        // ustalenie znaku pradu zmierzonego na podstawie znaku pwm
        if (set_value_new>0) current_measured = (float)meassured_current;
			else current_measured = (float) (-meassured_current);

        // wyznaczenie uchybu
        current_error = current_desired - current_measured;

        // wyznaczenie calki uchybu
        int_current_error =  int_current_error + INT_I_REG * current_error;		// 500Hz => 0.02s

        // przycinanie calki uchybu

        if (int_current_error >  MAX_PWM) int_current_error =  MAX_PWM;
        if (int_current_error < -MAX_PWM) int_current_error = -MAX_PWM;

        if (current_desired>=1)
        {
        	if (int_current_error<0) int_current_error = 0;
        }
        else if((current_desired<1) && (current_desired>-1))
        {
			int_current_error = 0;
        }
        else if (current_desired<=-1)
        {
        	if (int_current_error>0) int_current_error = 0;
        }


        // wyznaczenie nowego sterowania
        set_value_new = PROP_I_REG * current_error + int_current_error;

        display++;
        if ((display % 100)==0)
        {
            //display = 0;
            printf("joint 7:   meassured_current = %f,    int_current_error = %f,     set_value_new = %f \n", current_measured, int_current_error, set_value_new);
        }
        // DUNG END
    break;
    default: // w tym miejscu nie powinien wystapic blad zwiazany z
        // nieistniejacym numerem algorytmu
        set_value_new = 0; // zerowe nowe sterowanie
        break;
    }


    // ograniczenie na sterowanie
    if (set_value_new > MAX_PWM)
        set_value_new = MAX_PWM;
    if (set_value_new < -MAX_PWM)
        set_value_new = -MAX_PWM;

    /*
    #define MAXX_PWM 250
      // ograniczenie na sterowanie
      if (set_value_new > MAXX_PWM)
         set_value_new = MAXX_PWM;
      if (set_value_new < -MAXX_PWM)
         set_value_new = -MAXX_PWM;
         */

    //   if (set_value_new!=0.0) printf ("aa: %f\n", set_value_new);

    // ograniczenie przyrostu PWM
    // ma na celu zapobiegac osiaganiu zbyt duzych pradow we wzmacniaczach mocy
    if (set_value_new - set_value_old > IRP6_POSTUMENT_AXE8_MAX_PWM_INCREMENT)
        set_value_new = set_value_old + IRP6_POSTUMENT_AXE8_MAX_PWM_INCREMENT;
    if (set_value_new- set_value_old < -IRP6_POSTUMENT_AXE8_MAX_PWM_INCREMENT)
        set_value_new = set_value_old - IRP6_POSTUMENT_AXE8_MAX_PWM_INCREMENT;

    master.rb_obj->lock_mutex();

    master.rb_obj->step_data.desired_inc[7] = (float) step_new_pulse; // pozycja osi 0
    master.rb_obj->step_data.current_inc[7] = (short int) position_increment_new;
    master.rb_obj->step_data.pwm[7] = (float) set_value_new;
    master.rb_obj->step_data.uchyb[7]=(float) (step_new_pulse - position_increment_new);

    master.rb_obj->unlock_mutex();

    // if (set_value_new > 0.0) {
    //  cprintf("svn = %lf  pin = %lf\n",set_value_new, position_increment_new);
    // }

    // przepisanie nowych wartosci zmiennych do zmiennych przechowujacych wartosci poprzednie
    position_increment_old = position_increment_new;
    delta_eint_old = delta_eint;
    step_old_pulse = step_new_pulse;
    set_value_very_old = set_value_old;

    PWM_value = (int) set_value_new;


    //	printf("CC: PWM: %d, %d, %d, %d\n", PWM_value, meassured_current, reg_state, kk);

    // AUTOMAT ZABEZPIECZAJACY SILNIK CHWYTAKA PRZED PRZEGRZANIEM

    // wyznaczenie pradu na zalozonych horyzoncie wstecz
    if (master.step_counter > IRP6_POSTUMENT_GRIPPER_SUM_OF_CURRENTS_NR_OF_ELEMENTS)
    {
        sum_of_currents -= currents [current_index];
    }
    if (meassured_current>0)
    {
    sum_of_currents += meassured_current;
    } else {
    	sum_of_currents -= meassured_current;
    }

    currents [current_index] = meassured_current;

    current_index = ((++current_index)%IRP6_POSTUMENT_GRIPPER_SUM_OF_CURRENTS_NR_OF_ELEMENTS);

    //	printf("aa: %d, %d, %d\n",  sum_of_currents, meassured_current, kk);
    	//	printf("aa: %d\n", sum_of_currents);


    reg_state=next_reg_state;

    switch (reg_state)
    {
    case lib::GRIPPER_START_STATE:

        if (sum_of_currents > IRP6_POSTUMENT_GRIPPER_SUM_OF_CURRENTS_MAX_VALUE)
        {
            next_reg_state = lib::GRIPPER_BLOCKED_STATE;
            gripper_blocked_start_time = master.step_counter;
           // 				printf("gripper GRIPPER_BLOCKED_STATE state\n");
        }
        break;

    case lib::GRIPPER_BLOCKED_STATE:

        if (((master.step_counter - gripper_blocked_start_time) > GRIPPER_BLOCKED_TIME_PERIOD)
                && (!(sum_of_currents > IRP6_POSTUMENT_GRIPPER_SUM_OF_CURRENTS_MAX_VALUE*100)))
        {
            //			printf("gripper GRIPPER_START_STATE state\n");
            next_reg_state = lib::GRIPPER_START_STATE;
        }
        else
        {
            position_increment_old = 0;
            position_increment_new =  0;
            delta_eint_old =  0;
            delta_eint =  0;
            step_old_pulse = 0;
            step_new_pulse = 0;
            set_value_very_old =  0;
            set_value_old = 0;
            set_value_old =  0;
            set_value_new =  0;
        }
        break;

    default:
        break;
    }

    prev_reg_state=reg_state;

    return alg_par_status;

}
/*-----------------------------------------------------------------------*/

} // namespace common
} // namespace edp
} // namespace mrrocpp
