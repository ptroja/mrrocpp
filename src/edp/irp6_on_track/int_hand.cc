// ------------------------------------------------------------------------
//                            int_hand.cc
//
// Funkcja obslugi przerwania -- odczyt i zapis rejestrow sprzetowych dla robota irp6 on_track
//
// Ostatnia modyfikacja: 2006
// ------------------------------------------------------------------------

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <signal.h>
#include <ctype.h>
#include <sys/wait.h>
#include <sys/types.h>
#include <sys/neutrino.h>
#include <sys/sched.h>
#include <hw/inout.h>

#include "common/typedefs.h"
#include "common/impconst.h"
#include "common/com_buf.h"

// Klasa edp_irp6ot_effector.
#include "edp/irp6_on_track/edp_irp6ot_effector.h"
// Klasa hi_irp6ot.
#include "edp/irp6_on_track/hi_local.h"

// Zmienne globalne do komunikacji z procedura obslugi przerwan

extern struct sigevent event; // by y&w
extern volatile motor_data md; // Aktualne dane we/wy (obsluga przerwania)

extern edp_irp6ot_effector* master;   // Bufor polecen i odpowiedzi EDP_MASTER

// ------------------------------------------------------------------------
// #pragma off(check_stack);
// Obsluga przerwania sprzetowego

// UWAGA - zmienna ilosc serwomechanizmow w zaleznosci od tego czy gripper jest dolaczony czy nie


const struct sigevent *
            int_handler (void *arg, int int_id)
{
    status_of_a_dof robot_status[IRP6_ON_TRACK_NUM_OF_SERVOS];
    short int low_word, high_word;
    int i;

    WORD binary_output;
    WORD binary_input;
    BYTE analog_input[8];
    WORD tmp_buf; // do uzyku przy odczcie wejsc

    md.hardware_error = (uint64_t) ALL_RIGHT; // Nie ma bledow sprzetowych

    if(master->test_mode)
    {
        return (&event); // by Y&W
    }

    // INT_EMPTY obluga pusta
    // z zalozenia to pierwszy tryb w ktorym jest uruchomiona fukcja obslugi przewania  ze wzgledu na synchronizacje
    if (md.interrupt_mode == INT_EMPTY)
    {
        // konieczne dla skasowania przyczyny przerwania
        out8(ADR_OF_SERVO_PTR, INTERRUPT_GENERATOR_SERVO_PTR);
        in16(SERVO_REPLY_STATUS_ADR); // Odczyt stanu wylacznikow
        in16(SERVO_REPLY_INT_ADR);

        md.is_synchronised = true;
        for ( i = 0; i < master->number_of_servos; i++ )
        {
            out8(ADR_OF_SERVO_PTR, FIRST_SERVO_PTR + (BYTE)i);
            md.robot_status[i].adr_offset_plus_0 = robot_status[i].adr_offset_plus_0 = in16(SERVO_REPLY_STATUS_ADR); // Odczyt stanu wylacznikow
            md.robot_status[i].adr_offset_plus_2 = robot_status[i].adr_offset_plus_2 = in16(SERVO_REPLY_INT_ADR);

            if (i>5) // dla osi mechatronicznych
            {
                // jesli ktorakolwiek os jest niezsynchronizwana to i robot jest niezsynchronizowany
                if (!(robot_status[i].adr_offset_plus_0 & 0x0040))
                {
                    md.is_synchronised = false;
                }
            }
        }

        return (&event);
    }

    // INT_SERVOING tryb regulacji osi
    else if (md.interrupt_mode == INT_SERVOING)
    {

        // konieczne dla skasowania przyczyny przerwania
        out8(ADR_OF_SERVO_PTR, INTERRUPT_GENERATOR_SERVO_PTR);
        in16(SERVO_REPLY_STATUS_ADR); // Odczyt stanu wylacznikow
        in16(SERVO_REPLY_INT_ADR);

        for ( i = 0; i < master->number_of_servos; i++ )
        {
            // Odczyty stanu osi, polozenia oraz pradu wirnikow
            out8(ADR_OF_SERVO_PTR, FIRST_SERVO_PTR + (BYTE)i);
            md.robot_status[i].adr_offset_plus_0 = robot_status[i].adr_offset_plus_0 = in16(SERVO_REPLY_STATUS_ADR); // Odczyt stanu wylacznikow

            md.robot_status[i].adr_offset_plus_2 = robot_status[i].adr_offset_plus_2 = in16(SERVO_REPLY_INT_ADR);

            // Odczyt polozenia osi: slowo 32 bitowe - negacja licznikow 16-bitowych
            robot_status[i].adr_offset_plus_4 = 0xFFFF ^ in16(SERVO_REPLY_POS_LOW_ADR); // Mlodsze slowo 16-bitowe
            robot_status[i].adr_offset_plus_6 = 0xFFFF ^ in16(SERVO_REPLY_POS_HIGH_ADR);// Starsze slowo 16-bitowe

            if (i<6) // osie rezolwerowe
            {
                // Wycinanie niewykorzystywanych bitow mlodszego slowa licznika polozenia
                if (robot_status[i].adr_offset_plus_4 & 0x0800)
                    // Zly odczyt polozenia
                    robot_status[i].adr_offset_plus_4 &= 0x0000;
                else
                    robot_status[i].adr_offset_plus_4 &= 0x07FF;
            }

            md.robot_status[i].adr_offset_plus_4 = robot_status[i].adr_offset_plus_4;
            md.robot_status[i].adr_offset_plus_6 = robot_status[i].adr_offset_plus_6;

            low_word  = robot_status[i].adr_offset_plus_4;
            high_word = robot_status[i].adr_offset_plus_6;

            if (i<6) // osie z rezolwerami
            {
                md.current_absolute_position[i] =  ((uint32_t) (high_word* (int)(IRP6_ON_TRACK_AXE_0_TO_5_INC_PER_REVOLUTION))) + ((uint32_t) low_word);
            } else
            { // osie z enkoderami
                md.current_absolute_position[i] =  (((uint32_t) (high_word<<16)) & (0xFFFF0000)) | ((uint16_t) low_word);
            }

            //   md.robot_status[i].adr_offset_plus_6 = robot_status[i].adr_offset_plus_6;
            //   md.high_word = high_word;

            //  md.robot_status[i].adr_offset_plus_8 = robot_status[i].adr_offset_plus_8 = in16(SERVO_REPLY_REG_1_ADR); // Niewykorzystane
            //  md.robot_status[i].adr_offset_plus_a = robot_status[i].adr_offset_plus_a = in16(SERVO_REPLY_REG_2_ADR); // Niewykorzystane


            // Obsluga bledow
            if ( robot_status[i].adr_offset_plus_0 & 0x0100 )
                md.hardware_error |= (uint64_t) (SYNCHRO_ZERO << (5*i)); // Impuls zera rezolwera

            if (i>=6) //  sterowniki osi z mechatroniki z przekaznikami
            {
                if ( ~(robot_status[i].adr_offset_plus_0) & 0x4000 )
                    md.hardware_error |= (uint64_t) (SYNCHRO_SWITCH_ON << (5*i)); // Zadzialal wylacznik synchronizacji
            }  else
            {
                if ( robot_status[i].adr_offset_plus_0 & 0x8000 )
                    md.hardware_error |= (uint64_t) (SYNCHRO_SWITCH_ON << (5*i)); // Zadzialal wylacznik synchronizacji
            }

            // wylaczniki krancowe
            if (i>=6) //  sterowniki osi z mechatroniki z przekaznikami
            {
                if ( ~(robot_status[i].adr_offset_plus_0) & 0x1000 )
                {
                    //	out8(ADR_OF_SERVO_PTR, FIRST_SERVO_PTR + (BYTE)i);
                    //	out16(SERVO_COMMAND_1_ADR, RESET_ALARM); // Skasowanie alarmu i umozliwienie ruchu osi
                    md.hardware_error |= (uint64_t) (UPPER_LIMIT_SWITCH << (5*i)); // Zadzialal wylacznik "gorny" krancowy
                }
                else if ( ~(robot_status[i].adr_offset_plus_0) & 0x2000 )
                {
                    md.hardware_error |= (uint64_t) (LOWER_LIMIT_SWITCH << (5*i)); // Zadzialal wylacznik "dolny" krancowy
                }
            }  else
            {
                if ( ~(robot_status[i].adr_offset_plus_0) & 0x2000 )
                {
                    //	out8(ADR_OF_SERVO_PTR, FIRST_SERVO_PTR + (BYTE)i);
                    //	out16(SERVO_COMMAND_1_ADR, RESET_ALARM); // Skasowanie alarmu i umozliwienie ruchu osi
                    md.hardware_error |= (uint64_t) (UPPER_LIMIT_SWITCH << (5*i)); // Zadzialal wylacznik "gorny" krancowy
                }
                else if ( ~(robot_status[i].adr_offset_plus_0) & 0x4000 )
                {
                    md.hardware_error |= (uint64_t) (LOWER_LIMIT_SWITCH << (5*i)); // Zadzialal wylacznik "dolny" krancowy
                }
            }

            if ( robot_status[i].adr_offset_plus_0 & 0x0400 )
            {
                md.hardware_error |= (uint64_t) (OVER_CURRENT << (5*i));
                //     out8(ADR_OF_SERVO_PTR, FIRST_SERVO_PTR + (BYTE)i);
                //     out16(SERVO_COMMAND_1_ADR, RESET_ALARM); // Skasowanie alarmu i umozliwienie ruchu osi
            }
        }
        ;  // end: for

        if (robot_status[6].adr_offset_plus_0 & 0x0080) // czy wlaczono moc
        {
            md.is_power_on = true;
        } else
        {
            md.is_robot_blocked = true;
            md.is_power_on = false;
        }


        if ( md.hardware_error & HARDWARE_ERROR_MASK ) // wyciecie SYNCHRO_ZERO i SYNCHRO_SWITCH_ON
        {
            for ( i = 0; i < master->number_of_servos; i++ )
            {
                // Zapis wartosci zadanej wypelnienia PWM
                out8(ADR_OF_SERVO_PTR, FIRST_SERVO_PTR + (BYTE)i);
                out16(SERVO_COMMAND_1_ADR, STOP_MOTORS);
            }
            ; // end: for
            return (&event); // Yoyek & 7
        }

        for ( i = 0; i < master->number_of_servos; i++ )
        {
            // Zapis wartosci zadanej wypelnienia PWM
            out8(ADR_OF_SERVO_PTR, FIRST_SERVO_PTR + (BYTE)i);
            if (md.is_robot_blocked)
                md.robot_control[i].adr_offset_plus_0 &= 0xff00;
            out16(SERVO_COMMAND_1_ADR, md.robot_control[i].adr_offset_plus_0);
        }
        ; // end: for

        // odczyt wejsc analogowych (z przetwornikow) i binarnych
        out8(ADR_OF_SERVO_PTR, IN_OUT_PACKET);
        binary_input=in16(SERVO_REPLY_REG_1_ADR);
        tmp_buf=in16(SERVO_REPLY_STATUS_ADR);
        analog_input[0]=0x00ff & tmp_buf;
        analog_input[1]=((0xff00 & tmp_buf)>>8);
        tmp_buf=in16(SERVO_REPLY_INT_ADR);
        analog_input[2]=0x00ff & tmp_buf;
        analog_input[3]=((0xff00 & tmp_buf)>>8);
        tmp_buf=in16(SERVO_REPLY_POS_LOW_ADR);
        analog_input[4]=0x00ff & tmp_buf;
        analog_input[5]=((0xff00 & tmp_buf)>>8);
        tmp_buf=in16(SERVO_REPLY_POS_HIGH_ADR);
        analog_input[6]=0x00ff & tmp_buf;
        analog_input[7]=((0xff00 & tmp_buf)>>8);

        master->in_out_obj->set_input(&binary_input, analog_input);

        // ustawienie wyjscia o ile bylo takie zlecenie
        if (master->in_out_obj->set_output_flag)
        {
            master->in_out_obj->set_output_flag=false;
            master->in_out_obj->get_output(&binary_output);

            out8(ADR_OF_SERVO_PTR, IN_OUT_PACKET);
            // SERVO_COMMAND_2_ADR       0x212
            out16(SERVO_COMMAND_2_ADR, binary_output);
        }

        return (&event);
    } // end INT_SERVOING

    // INT_SINGLE_COMMAND do synchronizacji, inicjacji, etc.
    else if (md.interrupt_mode == INT_SINGLE_COMMAND)
    {

        // konieczne dla skasowania przyczyny przerwania
        out8(ADR_OF_SERVO_PTR, INTERRUPT_GENERATOR_SERVO_PTR);
        in16(SERVO_REPLY_STATUS_ADR); // Odczyt stanu wylacznikow
        in16(SERVO_REPLY_INT_ADR);

        out8(ADR_OF_SERVO_PTR, md.card_adress);
        out16(md	.register_adress, md.value);

        for ( i = 0; i < master->number_of_servos; i++ )
        {
            out8(ADR_OF_SERVO_PTR, FIRST_SERVO_PTR + (BYTE)i);
            md.robot_status[i].adr_offset_plus_0 = robot_status[i].adr_offset_plus_0 = in16(SERVO_REPLY_STATUS_ADR); // Odczyt stanu wylacznikow
            md.robot_status[i].adr_offset_plus_2 = robot_status[i].adr_offset_plus_2 = in16(SERVO_REPLY_INT_ADR);
        }
        md.interrupt_mode=INT_EMPTY; // aby tylko raz wyslac polecenie

        return (&event);
    }

    // INT_CHECK_STATE do odczytu stanu z adresu 0x220
    else if (md.interrupt_mode == INT_CHECK_STATE)
    {

        // konieczne dla skasowania przyczyny przerwania
        out8(ADR_OF_SERVO_PTR, INTERRUPT_GENERATOR_SERVO_PTR);
        in16(SERVO_REPLY_STATUS_ADR); // Odczyt stanu wylacznikow
        in16(SERVO_REPLY_INT_ADR);

        for ( i = 0; i < master->number_of_servos; i++ )
        {
            out8(ADR_OF_SERVO_PTR, FIRST_SERVO_PTR + (BYTE)i);
            md.robot_status[i].adr_offset_plus_0 = robot_status[i].adr_offset_plus_0 = in16(SERVO_REPLY_STATUS_ADR); // Odczyt stanu wylacznikow
            md.robot_status[i].adr_offset_plus_2 = robot_status[i].adr_offset_plus_2 = in16(SERVO_REPLY_INT_ADR);
        }
        md.interrupt_mode=INT_EMPTY; // aby tylko raz sprawdzic stan

        return (&event);
    }


    // Zakonczenie obslugi przerwania ze wzbudzeniem posrednika (proxy)

    return (&event);// Yoyek & wojtek

}
; // // end: int_handler()

// #pragma on(check_stack);
// ------------------------------------------------------------------------
