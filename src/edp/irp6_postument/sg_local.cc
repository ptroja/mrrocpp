/* --------------------------------------------------------------------- */
/*                          SERVO_GROUP Process                          */
// ostatnia modyfikacja - styczen 2005
/* --------------------------------------------------------------------- */

#include <unistd.h>

#include "common/typedefs.h"
#include "common/impconst.h"
#include "common/com_buf.h"


// Klasa edp_irp6p_effector.
#include "edp/irp6_postument/edp_irp6p_effector.h"
// Klasa hi_irp6p.
#include "edp/irp6_postument/hi_local.h"
// Klasa irp6p_servo_buffer.
#include "edp/irp6_postument/sg_local.h"

namespace mrrocpp {
namespace edp {
namespace common {

// extern uint64_t kk;				  // numer pomiaru od momentu startu pomiarow



/*-----------------------------------------------------------------------*/
BYTE irp6p_servo_buffer::Move_a_step (void)
{
    // wykonac ruch o krok nie reagujac na SYNCHRO_SWITCH ora SYNCHRO_ZERO

    Move_1_step ();
    if (master.is_synchronised())
    {// by Y aktualizacja transformera am jedynie sens po synchronizacji (kiedy robot zna swoja pozycje)
        // by Y - do dokonczenia
        for (int i=0; i < IRP6_POSTUMENT_NUM_OF_SERVOS; i++)
        {
            if (!(master.test_mode))
            {
                switch (i)
                {
                case IRP6P_GRIPPER_CATCH_AXE:
                    master.update_servo_current_motor_pos_abs(hi->get_position(i)*(2*M_PI)/IRP6_POSTUMENT_AXE_7_INC_PER_REVOLUTION, i);
                    break;
                case IRP6P_GRIPPER_TURN_AXE:
                    master.update_servo_current_motor_pos_abs(hi->get_position(i)*(2*M_PI)/IRP6_POSTUMENT_AXE_6_INC_PER_REVOLUTION, i);
                    break;
                default:
                    master.update_servo_current_motor_pos_abs(hi->get_position(i)*(2*M_PI)/IRP6_POSTUMENT_AXE_0_TO_5_INC_PER_REVOLUTION, i);
                    break;
                }
            }

            /*
            	if (i==6) 
        {
              master.update_servo_current_motor_pos(regulator_ptr[i]->get_position_inc(0)*2*M_PI/IRP6_POSTUMENT_AXE_7_INC_PER_REVOLUTION,  i);
        } else if (i==5) 
        {		
            master.update_servo_current_motor_pos(regulator_ptr[i]->get_position_inc(0)*2*M_PI/IRP6_POSTUMENT_AXE_6_INC_PER_REVOLUTION,  i);
        } else 
        {		
            master.update_servo_current_motor_pos(regulator_ptr[i]->get_position_inc(0)*2*M_PI/IRP6_POSTUMENT_AXE_0_TO_5_INC_PER_REVOLUTION,  i);
        } 
            */
        }

        master.servo_joints_and_frame_actualization_and_upload();// by Y - aktualizacja trasformatora
    }
    return convert_error();
}
; // end: servo_buffer::Move_a_step
/*-----------------------------------------------------------------------*/


/*-----------------------------------------------------------------------*/
irp6p_servo_buffer::irp6p_servo_buffer ( edp_irp6p_effector &_master) : servo_buffer(_master), master(_master)
{

    hi = new hi_irp6p(_master);

    // utworzenie tablicy regulatorow
    // Serwomechanizm 1

    // regulator_ptr[1] = new NL_regulator_2 (0, 0, 0.71, 13./4, 12.57/4, 0.35);
    // kolumna dla irp6 postument
    regulator_ptr[0] = new NL_regulator_2_irp6p (0, 0, 0.429, 6.834, 6.606, 0.35, master); // kolumna dla irp6 postument

    regulator_ptr[1] = new NL_regulator_3_irp6p (0, 0, 0.64, 9.96/4, 9.54/4, 0.35, master);

    // regulator_ptr[3] = new NL_regulator_4 (0, 0, 0.62, 9.85/4, 9.39/4, 0.35);
    regulator_ptr[2] = new NL_regulator_4_irp6p (0, 0, 0.333, 5.693, 5.427, 0.35, master);

    regulator_ptr[3] = new NL_regulator_5_irp6p (0, 0, 0.56, 7.98/2, 7.55/2, 0.35, master);

    // regulator_ptr[5] = new NL_regulator_6 (0, 0, 0.3079*2, 0.6, 0.6, 0.35);
    regulator_ptr[4] = new NL_regulator_6_irp6p (0, 0, 0.39, 8.62/2., 7.89/2., 0.35, master);
    // regulator_ptr[0] = new NL_regulator_1 (0, 0, 0.64, 16.61/5., 15.89/5., 0.35);

    regulator_ptr[5] = new NL_regulator_7_irp6p (0, 0, 0.39, 8.62/2., 7.89/2., 0.35, master);

    // chwytak
    regulator_ptr[6] = new NL_regulator_8_irp6p (0, 0, 0.39, 8.62/2., 7.89/2., 0.35, master);


    send_after_last_step = false;
    clear_reply_status();
    clear_reply_status_tmp();

    for (int j = 0; j < IRP6_POSTUMENT_NUM_OF_SERVOS; j++)
    {

        command.parameters.move.abs_position[j]=0.0;
    }
    ; // end: for


}
; // end: servo_buffer::servo_buffer
/*-----------------------------------------------------------------------*/

/*-----------------------------------------------------------------------*/
/*
void irp6p_servo_buffer::synchronise (void) {
      
 regulator* crp = NULL; // wskaznik aktualnie synchronizowanego napedu
 
 int j;
 
 double synchro_step = 0.0;   // zadany przyrost polozenia
 
	if(master.test_mode) {
		// W.S. Tylko przy testowaniu
		clear_reply_status();  
		clear_reply_status_tmp();
		reply_to_EDP_MASTER(); 
		return;
	}
 
	// synchronizacja wlasciwa
	hi->synchronise_via_lm629();
 
	// zerowanie regulatorow
	for (j = 0; j < IRP6_POSTUMENT_NUM_OF_SERVOS; j++)
	{
		crp = regulator_ptr[j];
		crp->clear_regulator();
		hi->reset_position(j);
	}
 
 
	// zatrzymanie na chwile robota
	for (j = 0; j < IRP6_POSTUMENT_NUM_OF_SERVOS; j++) 
	{
		synchro_step=0.0;
		crp = regulator_ptr[j];
		crp->insert_new_step(synchro_step);
	};
	
	for (j = 0; j < 25; j++) 
		Move_1_step();
	
	kk = 0;
	clear_reply_status();  
	clear_reply_status_tmp();
	
	reply_to_EDP_MASTER();
	return;
	
}; // end: servo_buffer::synchronise()
*/
/*-----------------------------------------------------------------------*/



/*-----------------------------------------------------------------------*/

void irp6p_servo_buffer::synchronise (void)
{

    const int NS = 10;     // liczba krokow rozpedzania/hamowania
    regulator* crp = NULL; // wskaznik aktualnie synchronizowanego napedu

    double synchro_step = 0.0;   // zadany przyrost polozenia

    if(master.test_mode)
    {
        // W.S. Tylko przy testowaniu
        clear_reply_status();
        clear_reply_status_tmp();
        reply_to_EDP_MASTER();
        return;
    }

    for (int j = 0; j < (master.number_of_servos); j++)
    {

        command.parameters.move.abs_position[j]=0.0;
    }
    ; // end: for


    // szeregowa synchronizacja serwomechanizmow
    for (int k = 0; k < (master.number_of_servos); k++)
    {
        int j = ((k+IRP6P_SYN_INIT_AXE)%(master.number_of_servos));

        // printf("os synchronizopwana: %d \n",j);
        for (int l= 0; l < (master.number_of_servos); l++)
        {
            int i = ((l+IRP6P_SYN_INIT_AXE)%(master.number_of_servos));
            // zerowy przyrost polozenia dla wszystkich napedow procz j-tego
            if ( i == j)
            {
                crp = regulator_ptr[i];
                // W.S.        crp->insert_new_step(SYNCHRO_STEP_COARSE);
                switch (i)
                {
                case IRP6P_GRIPPER_CATCH_AXE:
                    synchro_step = IRP6_POSTUMENT_AXE_7_SYNCHRO_STEP_COARSE/NS;
                    break;
                case IRP6P_GRIPPER_TURN_AXE:
                    synchro_step = IRP6_POSTUMENT_AXE_6_SYNCHRO_STEP_COARSE/NS;
                    break;
                default:
                    synchro_step = IRP6_POSTUMENT_AXE_0_TO_5_SYNCHRO_STEP_COARSE/NS;
                    break;
                }
                crp->insert_new_step(synchro_step);
            }
            else
            {
                regulator_ptr[i]->insert_new_step(0.0);
            }
        }
        ; // end: for

        clear_reply_status();
        clear_reply_status_tmp();

        synchro_step=0.0;

        // ruch do wykrycia wylacznika synchronizacji
        for (;;)
        {
            do
            {
                switch (j)
                {
                case IRP6P_GRIPPER_CATCH_AXE:
                    if (synchro_step > IRP6_POSTUMENT_AXE_7_SYNCHRO_STEP_COARSE )
                    {
                        synchro_step += IRP6_POSTUMENT_AXE_7_SYNCHRO_STEP_COARSE/NS;
                        crp->insert_new_step(synchro_step);
                    }
                    break;
                case IRP6P_GRIPPER_TURN_AXE:
                    if (synchro_step > IRP6_POSTUMENT_AXE_6_SYNCHRO_STEP_COARSE )
                    {
                        synchro_step += IRP6_POSTUMENT_AXE_6_SYNCHRO_STEP_COARSE/NS;
                        crp->insert_new_step(synchro_step);
                    }
                    break;
                default:
                    if (synchro_step > IRP6_POSTUMENT_AXE_0_TO_5_SYNCHRO_STEP_COARSE )
                    {
                        synchro_step += IRP6_POSTUMENT_AXE_0_TO_5_SYNCHRO_STEP_COARSE/NS;
                        crp->insert_new_step(synchro_step);
                    }
                    break;
                }
                //		printf("aaa: %d, %x\n", j, reply_status_tmp.error0);
            }
            while ( Move_1_step() == NO_ERROR_DETECTED ); // end: while
            //		printf("aaa: %d, %x\n", j, reply_status_tmp.error0);
            // analiza przyslanego bledu (czy wjechano na wylacznik synchronizacji?)
            // jezeli nie, to blad
            switch ( (reply_status_tmp.error0 >> (5*j)) & 0x000000000000001FULL )
            {
            case SYNCHRO_SWITCH_ON:
                //  printf("aaa: SYNCHRO_SWITCH_ON\n");
            case SYNCHRO_SWITCH_ON_AND_SYNCHRO_ZERO:
                // cprintf("B=%lx\n", reply_status_tmp.error0);
                //		printf("aaa: SYNCHRO_SWITCH_ON_AND_SYNCHRO_ZERO\n");
                break;
            case ALL_RIGHT:
            case SYNCHRO_ZERO:
                //     printf("aaa: SYNCHRO_ZERO\n");
                continue;
            default:
                //    printf("aaa: default\n");
                // awaria w trakcie synchronizacji
                convert_error();
                reply_status.error0 = reply_status_tmp.error0 | SYNCHRO_SWITCH_EXPECTED;
                reply_status.error1 = reply_status_tmp.error1;
                clear_reply_status_tmp();
                reply_to_EDP_MASTER();
                return;
            }
            ; // end: switch
            break;
        }
        ; // end: for (;;)

        //	printf("przed clear_reply_status \n");

        clear_reply_status();
        clear_reply_status_tmp();

        // zatrzymanie na chwile robota
        synchro_step=0.0;
        crp->insert_new_step(synchro_step);
        for (int i = 0; i < 250; i++)
        {
            Move_1_step();
            //  printf("aabb: %d, %x\n", j, reply_status_tmp.error0);
            switch ( (reply_status_tmp.error0 >> (5*j)) & 0x000000000000001FULL )
            {
            case SYNCHRO_SWITCH_ON:
            case SYNCHRO_SWITCH_ON_AND_SYNCHRO_ZERO:
            case ALL_RIGHT:
            case SYNCHRO_ZERO:
                continue;
            default:
                // awaria w trakcie stania
                convert_error();
                reply_status.error0 = reply_status_tmp.error0 | SYNCHRO_DELAY_ERROR;
                reply_status.error1 = reply_status_tmp.error1;
                clear_reply_status_tmp();
                reply_to_EDP_MASTER();
                return;
            }
            ; // end: switch
        }
        ; // end: for (i...)
        // cprintf("C=%lx\n", reply_status_tmp.error0);

        clear_reply_status();
        clear_reply_status_tmp();

        // zjazd z wylacznika synchronizacji
        // W.S.  crp->insert_new_step(SYNCHRO_STEP_FINE);
        switch (j)
        {
        case IRP6P_GRIPPER_CATCH_AXE:
            synchro_step = -IRP6_POSTUMENT_AXE_7_SYNCHRO_STEP_FINE/NS;
            break;
        case IRP6P_GRIPPER_TURN_AXE:
            synchro_step = -IRP6_POSTUMENT_AXE_6_SYNCHRO_STEP_FINE/NS;
            break;
        default:
            synchro_step = -IRP6_POSTUMENT_AXE_0_TO_5_SYNCHRO_STEP_FINE/NS;
            break;
        }

        crp->insert_new_step(synchro_step);

        // wlaczenie sledzenia zera rezolwera (synchronizacja osi)
        hi->start_synchro (j);
        delay(1);
        Move_1_step();
        while (1)
        {
            //  		printf("babb: %d\n", j);
            Move_1_step();
            // W.S. -----------------------------------------------------
            //	printf("ccc: %d\n", j);
            switch (j)
            {
            case IRP6P_GRIPPER_CATCH_AXE:
                if (synchro_step < -IRP6_POSTUMENT_AXE_7_SYNCHRO_STEP_FINE )
                {
                    synchro_step -= IRP6_POSTUMENT_AXE_7_SYNCHRO_STEP_FINE / NS;
                    crp->insert_new_step(synchro_step);
                }
                break;
            case IRP6P_GRIPPER_TURN_AXE:
                if (synchro_step < -IRP6_POSTUMENT_AXE_6_SYNCHRO_STEP_FINE )
                {
                    synchro_step -= IRP6_POSTUMENT_AXE_6_SYNCHRO_STEP_FINE / NS;
                    crp->insert_new_step(synchro_step);
                }
                break;
            default:
                if (synchro_step < -IRP6_POSTUMENT_AXE_0_TO_5_SYNCHRO_STEP_FINE )
                {
                    synchro_step -= IRP6_POSTUMENT_AXE_0_TO_5_SYNCHRO_STEP_FINE / NS;
                    crp->insert_new_step(synchro_step);
                }
                break;
            }



            // W.S. -----------------------------------------------------
            //    	   printf("bbbb if: %llx\n", ((reply_status_tmp.error0 >> (5*j)) & 0x000000000000001FULL));
            switch ((reply_status_tmp.error0 >> (5*j)) & 0x000000000000001FULL)
            {
            case SYNCHRO_SWITCH_ON:
                //    	printf("bcbb:�SYNCHRO_SWITCH_ON\n");
            case SYNCHRO_SWITCH_ON_AND_SYNCHRO_ZERO:
                //     	printf("bfbb: SYNCHRO_SWITCH_ON_AND_SYNCHRO_ZERO\n");
                continue;
            default:
                //   	printf("baabb: default\n");
                break;
            }
            ; // end: switch
            break;
        }
        ; // end: while
        //	 printf("D\n ");

        // analiza powstalej sytuacji (czy zjechano z wylacznika synchronizacji)
        // jezeli nie, to blad
        switch ( ((reply_status_tmp.error0 >> (5*j)) & 0x000000000000001FULL) )
        {
        case SYNCHRO_ZERO: // zjechano z wylacznika synchronizacji i SYNCHRO_ZERO jest od razu
            //     printf("SYNCHRO_ZERO\n");
            hi->finish_synchro (j);

            //	 printf("SYNCHRO_ZERO\n");
            hi->reset_position(j);
            crp->clear_regulator();
            delay(1);
            continue;
        case OK:
            // ruch do wykrycia zera rezolwera
            //    printf("OK\n");
            for (;;)
            {
                Move_1_step();
                //        printf("OK Move_1_step\n");
                //      if ( ((reply_status_tmp.error0 >> (5*j)) & 0xCE739CE739CE739FULL) != OK)
                // by Y - wyciecie SYNCHRO_SWITCH_ON - ze wzgledu na wystepujace drgania
                if ( ((reply_status_tmp.error0 >> (5*j)) & 0xCE739CE739CE739DULL) != OK)
                {
                    //   printf("OK os: %d, if: %llx, %llx\n", j, reply_status_tmp.error0, ((reply_status_tmp.error0 >> (5*j)) & 0xCE739CE739CE739FULL));
                    // Usuniecie bitow SYNCHRO_ZERO i SYNCHRO_SWITCH_ON z wszystkich osi
                    // oprocz synchronizowanej
                    // osie zsynchronizowane nie sa analizowane
                    break;
                }
            }
            ; // end: for (;;)
            //     if ( ((reply_status_tmp.error0 >> (5*j)) & 0x000000000000001FULL) != SYNCHRO_ZERO) {
            // by Y - wyciecie SYNCHRO_SWITCH_ON
            if ( ((reply_status_tmp.error0 >> (5*j)) & 0x000000000000001DULL) != SYNCHRO_ZERO)
            {
                // 	  printf("OK convert_error: %llx\n", ((reply_status_tmp.error0 >> (5*j)) & 0x000000000000001FULL));
                convert_error();
                reply_status.error0 = reply_status_tmp.error0 | SYNCHRO_ERROR;
                reply_status.error1 = reply_status_tmp.error1;
                clear_reply_status_tmp();
                // Wypelnic servo_data
                reply_to_EDP_MASTER();
                return;
            }
            else
            {
                hi->finish_synchro(j);
                hi->reset_position(j);
                crp->clear_regulator();
                delay(1);
                continue;
            }
            ; // end: else
        default:
            //    	printf(" default error\n");
            // awaria w trakcie synchronizacji
            convert_error();
            reply_status.error0 = reply_status_tmp.error0 | SYNCHRO_ERROR;
            reply_status.error1 = reply_status_tmp.error1;
            clear_reply_status_tmp();
            // Wypelnic servo_data
            reply_to_EDP_MASTER();
            return;
        }
        ; // end: switch
        // zakonczenie synchronizacji danej osi i przejscie do trybu normalnego
    }
    ; // end: for (int j = 0; j < IRP6_POSTUMENT_NUM_OF_SERVOS)

    // zatrzymanie na chwile robota
    for (int k = 0; k < (master.number_of_servos); k++)
    {
        int j = ((k+IRP6P_SYN_INIT_AXE)%(master.number_of_servos));
        synchro_step=0.0;
        crp = regulator_ptr[j];
        crp->insert_new_step(synchro_step);
    };
    for (int i = 0; i < 25; i++)
        Move_1_step();

    // kk = 0;

    // printf("koniec synchro\n");
    reply_to_EDP_MASTER();
    return;

}
; // end: servo_buffer::synchronise()

/*-----------------------------------------------------------------------*/



/*-----------------------------------------------------------------------*/
irp6p_servo_buffer::~irp6p_servo_buffer(void)
{}
; // end: regulator_group::~regulator_group
/*-----------------------------------------------------------------------*/


/*-----------------------------------------------------------------------*/
void irp6p_servo_buffer::get_all_positions (void)
{
    // Przepisanie aktualnych polozen servo do pakietu wysylkowego
    for (int i = 0; i < IRP6_POSTUMENT_NUM_OF_SERVOS; i++)
    {

        switch (i)
        {
        case IRP6P_GRIPPER_CATCH_AXE:
            servo_data.abs_position[i]  = hi->get_position(i)*(2*M_PI)/IRP6_POSTUMENT_AXE_7_INC_PER_REVOLUTION;
            break;
        case IRP6P_GRIPPER_TURN_AXE:
            servo_data.abs_position[i]  = hi->get_position(i)*(2*M_PI)/IRP6_POSTUMENT_AXE_6_INC_PER_REVOLUTION;
            break;
        default:
            servo_data.abs_position[i]  = hi->get_position(i)*(2*M_PI)/IRP6_POSTUMENT_AXE_0_TO_5_INC_PER_REVOLUTION;
            break;
        }


        // przyrost polozenia w impulsach
        servo_data.position[i]  = regulator_ptr[i]->get_position_inc(1);
        servo_data.current[i]   = regulator_ptr[i]->get_meassured_current();
        servo_data.PWM_value[i] = regulator_ptr[i]->get_PWM_value();
        servo_data.algorithm_no[i] = regulator_ptr[i]->get_algorithm_no();
        servo_data.algorithm_parameters_no[i] = regulator_ptr[i]->get_algorithm_parameters_no();
    }
    ; // end: for

    // przepisanie stanu regulatora chwytaka do bufora odpowiedzi dla EDP_master
    servo_data.gripper_reg_state = regulator_ptr[6]->get_reg_state();

}
; // end: output_buffer::get_all_positions
/*-----------------------------------------------------------------------*/

/*-----------------------------------------------------------------------*/
uint64_t irp6p_servo_buffer::compute_all_set_values (void)
{
    // obliczenie nastepnej wartosci zadanej dla wszystkich napedow
    uint64_t status = OK; // kumuluje numer bledu

 
    for (int j = 0; j < IRP6_POSTUMENT_NUM_OF_SERVOS; j++)
    {
        if (master.test_mode)
        {
            switch (j)
            {
            case IRP6P_GRIPPER_CATCH_AXE:
                regulator_ptr[j]->insert_new_pos_increment(regulator_ptr[j]->return_new_step()
                        *IRP6_POSTUMENT_AXE_7_INC_PER_REVOLUTION/(2*M_PI));
                break;
            case IRP6P_GRIPPER_TURN_AXE:
                regulator_ptr[j]->insert_new_pos_increment(regulator_ptr[j]->return_new_step()
                        *IRP6_POSTUMENT_AXE_6_INC_PER_REVOLUTION/(2*M_PI));
                break;
            default:
                regulator_ptr[j]->insert_new_pos_increment(regulator_ptr[j]->return_new_step()
                        *IRP6_POSTUMENT_AXE_0_TO_5_INC_PER_REVOLUTION/(2*M_PI));
                break;
            }
        }
        else
        {
            regulator_ptr[j]->insert_meassured_current(hi->get_current(j));
            regulator_ptr[j]->insert_new_pos_increment(hi->get_increment(j));
        }
        // obliczenie nowej wartosci zadanej dla napedu
        status |= ((uint64_t) regulator_ptr[j]->compute_set_value()) << 2*j;
        // przepisanie obliczonej wartosci zadanej do hardware interface
        hi->insert_set_value(j, regulator_ptr[j]->get_set_value());
    }
    ; // end: for
    return status;
}
; // end: servo_buffer::compute_all_set_values
/*-----------------------------------------------------------------------*/



servo_buffer* return_created_servo_buffer (edp_irp6s_and_conv_effector &_master)
                    {
                        return new irp6p_servo_buffer ((edp_irp6p_effector &)(_master));
                    };

} // namespace common
} // namespace edp
} // namespace mrrocpp

