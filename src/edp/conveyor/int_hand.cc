// ------------------------------------------------------------------------
//                            int_hand.cc
//
// Funkcja obslugi przerwania -- odczyt i zapis rejestrow sprzetowych dla robota conveyor
//
// Ostatnia modyfikacja: 2005
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
#include <sys/iofunc.h>// Y&7


#include "common/typedefs.h"
#include "common/impconst.h"
#include "common/com_buf.h"

// Klasa edp_conveyor_effector.
#include "edp/conveyor/edp_conveyor_effector.h"
// Klasa hi_conv.
#include "edp/conveyor/hi_local.h"

// Zmienne globalne do komunikacji z procedura obslugi przerwan
extern int int_id;      // Identyfikator zwiazany z procedura obslugi przerwania
extern struct sigevent event; // by y&w
extern volatile motor_data md; // Aktualne dane we/wy (obsluga przerwania)

extern edp_conveyor_effector* master;

// ------------------------------------------------------------------------
// #pragma off(check_stack);
// Obsluga przerwania sprzetowego

const struct sigevent *
            int_handler (void *arg, int int_id)
{
    status_of_a_dof robot_status[CONVEYOR_NUM_OF_SERVOS];
    short int low_word, high_word;

    md.hardware_error = (uint64_t) ALL_RIGHT; // Nie ma bledow sprzetowych

    if(master->test_mode)
    {
        return (&event);// by Y&W
    }

    // INT_EMPTY obluga pusta
    if (md.interrupt_mode==INT_EMPTY)
    {
        // konieczne dla skasowania przyczyny przerwania
        out8(ADR_OF_SERVO_PTR, FIRST_SERVO_PTR + CONVEYOR_SERVO_NR);
        md.robot_status[0].adr_offset_plus_0 = robot_status[0].adr_offset_plus_0 = in16(SERVO_REPLY_STATUS_ADR); // Odczyt stanu wylacznikow
        md.robot_status[0].adr_offset_plus_2 = robot_status[0].adr_offset_plus_2 = in16(SERVO_REPLY_INT_ADR);

        return (&event);
    }

    // INT_SERVOING tryb regulacji osi
    else if (md.interrupt_mode==INT_SERVOING)
    {
        // Odczyty stanu osi, polozenia oraz pradu wirnikow
        out8(ADR_OF_SERVO_PTR, FIRST_SERVO_PTR + (BYTE)CONVEYOR_SERVO_NR);
        md.robot_status[0].adr_offset_plus_0 = robot_status[0].adr_offset_plus_0 = in16(SERVO_REPLY_STATUS_ADR); // Odczyt stanu wylacznikow

        md.robot_status[0].adr_offset_plus_2 = robot_status[0].adr_offset_plus_2 = in16(SERVO_REPLY_INT_ADR);

        // Odczyt polozenia osi: slowo 32 bitowe - negacja licznikow 16-bitowych
        robot_status[0].adr_offset_plus_4 = 0xFFFF ^ in16(SERVO_REPLY_POS_LOW_ADR);  // Mlodsze slowo 16-bitowe
        robot_status[0].adr_offset_plus_6 = 0xFFFF ^ in16(SERVO_REPLY_POS_HIGH_ADR); // Starsze slowo 16-bitowe

        md.robot_status[0].adr_offset_plus_4 = robot_status[0].adr_offset_plus_4;
        md.robot_status[0].adr_offset_plus_6 = robot_status[0].adr_offset_plus_6;

        low_word  = robot_status[0].adr_offset_plus_4;
        high_word = robot_status[0].adr_offset_plus_6;

        // Obliczenie polozenia
        md.current_absolute_position[0] =  (((uint32_t) (high_word<<16)) & (0xFFFF0000)) | ((uint16_t) low_word);

        //   md.robot_status[i].adr_offset_plus_6 = robot_status[i].adr_offset_plus_6;
        //   md.high_word = high_word;

        //  md.robot_status[i].adr_offset_plus_8 = robot_status[i].adr_offset_plus_8 = in16(SERVO_REPLY_REG_1_ADR); // Niewykorzystane
        //  md.robot_status[i].adr_offset_plus_a = robot_status[i].adr_offset_plus_a = in16(SERVO_REPLY_REG_2_ADR); // Niewykorzystane

        // Obsluga bledow

        if ( robot_status[0].adr_offset_plus_0 & 0x0100 )
            md.hardware_error |= (uint64_t) (SYNCHRO_ZERO); // Impuls zera rezolwera

        if ( robot_status[0].adr_offset_plus_0 & 0x4000 )
            md.hardware_error |= (uint64_t) (SYNCHRO_SWITCH_ON); // Zadzialal wylacznik synchronizacji


        if ( robot_status[0].adr_offset_plus_0 & 0x0400 )
        {
            md.hardware_error |= (uint64_t) (OVER_CURRENT);
            //     out8(ADR_OF_SERVO_PTR, FIRST_SERVO_PTR + (BYTE)i);
            //     out16(SERVO_COMMAND_1_ADR, RESET_ALARM); // Skasowanie alarmu i umozliwienie ruchu osi
        }

        if (robot_status[0].adr_offset_plus_0 & 0x0080) // czy wlaczono moc
        {
            md.is_power_on = true;
        } else
        {
            md.is_power_on = false;
            md.is_robot_blocked = true;
        }


        if ( md.hardware_error & HARDWARE_ERROR_MASK ) // wyciecie SYNCHRO_ZERO i SYNCHRO_SWITCH_ON
        {

            // Zapis wartosci zadanej wypelnienia PWM
            out8(ADR_OF_SERVO_PTR, FIRST_SERVO_PTR + (BYTE)CONVEYOR_SERVO_NR);
            out16(SERVO_COMMAND_1_ADR, STOP_MOTORS);

            return (&event); // Yoyek & 7
        }


        // Zapis wartosci zadanej wypelnienia PWM
        out8(ADR_OF_SERVO_PTR, FIRST_SERVO_PTR + (BYTE)CONVEYOR_SERVO_NR);
        if (md.is_robot_blocked)
            md.robot_control[0].adr_offset_plus_0 &= 0xff00;
        out16(SERVO_COMMAND_1_ADR, md.robot_control[0].adr_offset_plus_0);

        return (&event);
    } // end INT_SERVOING

    // INT_SINGLE_COMMAND do synchronizacji, inicjacji, etc.
    else if (md.interrupt_mode==INT_SINGLE_COMMAND)
    {
        out8(ADR_OF_SERVO_PTR, md.card_adress);
        out16(md	.register_adress, md.value);
        // konieczne dla skasowania przyczyny przerwania

        out8(ADR_OF_SERVO_PTR, FIRST_SERVO_PTR + (BYTE)CONVEYOR_SERVO_NR);
        md.robot_status[0].adr_offset_plus_0 = robot_status[0].adr_offset_plus_0 = in16(SERVO_REPLY_STATUS_ADR); // Odczyt stanu wylacznikow
        md.robot_status[0].adr_offset_plus_2 = robot_status[0].adr_offset_plus_2 = in16(SERVO_REPLY_INT_ADR);

        md.interrupt_mode=INT_EMPTY; // aby tylko raz wyslac polecenie
        return (&event);
    }

    // INT_CHECK_STATE do odczytu stanu z adresu 0x220
    else if (md.interrupt_mode==INT_CHECK_STATE)
    {
        // konieczne dla skasowania przyczyny przerwania
        out8(ADR_OF_SERVO_PTR, FIRST_SERVO_PTR + (BYTE)CONVEYOR_SERVO_NR);
        md.robot_status[0].adr_offset_plus_0 = robot_status[0].adr_offset_plus_0 = in16(SERVO_REPLY_STATUS_ADR); // Odczyt stanu wylacznikow
        md.robot_status[0].adr_offset_plus_2 = robot_status[0].adr_offset_plus_2 = in16(SERVO_REPLY_INT_ADR);

        md.interrupt_mode=INT_EMPTY; // aby tylko raz sprawdzic stan
        return (&event);
    }


    // Zakonczenie obslugi przerwania ze wzbudzeniem posrednika (proxy)

    return (&event);// Yoyek & wojtek

}
; // end: int_handler()

// #pragma on(check_stack);
// ------------------------------------------------------------------------
