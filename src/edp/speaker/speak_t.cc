// ------------------------------------------------------------------------
// transformation thread by Y
// ostatnia modyfikacja: styczen 2005
// ------------------------------------------------------------------------

/********************************* INCLUDES *********************************/
#include <stdio.h>
#include <string.h>
#include <signal.h>
#include <stdlib.h>

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"
#include "lib/srlib.h"
#include "lib/mis_fun.h"

// Klasa edp_speaker_effector.
#include "edp/speaker/edp_speaker_effector.h"

namespace mrrocpp {
namespace edp {
namespace speaker {

void * effector::speak_thread_start(void* arg)
{
//	 edp_irp6s_and_conv_effector *master = (edp_irp6s_and_conv_effector *) arg;

	 return static_cast<effector*> (arg)->speak_thread(arg);
}


void * effector::speak_thread(void *arg)
{
    lib::set_thread_priority(pthread_self() , MAX_PRIORITY-10);

    if( init() == -1)
    {
        initialize_incorrect=1;
    }
    else
    {
        initialize_incorrect=0;
    }

    while(1)
    {
        mt_tt_obj->trans_t_wait_for_master_order();// oczekiwanie na zezwolenie ruchu od edp_master

        // przekopiowanie instrukcji z bufora watku komunikacji z ECP (edp_master)
        memcpy( &(current_instruction), &(new_instruction), sizeof(new_instruction) );
        try
        {

            switch (mt_tt_obj->trans_t_task)
            {
            case common::MT_GET_ARM_POSITION:
                // master.get_arm_position(mt_tt_obj->trans_t_tryb, &(master.current_instruction));
                get_spoken(mt_tt_obj->trans_t_tryb, &(current_instruction)); // MAC7
                mt_tt_obj->trans_t_to_master_order_status_ready();
                break;
            case common::MT_MOVE_ARM:
                // master.move_arm(&(master.current_instruction)); 	 // wariant dla watku edp_trans_t
                mt_tt_obj->trans_t_to_master_order_status_ready();
                speak(&(current_instruction)); // MAC7
                break;
            default: // blad: z reply_type wynika, e odpowied nie ma zawiera narzedzia
                break;
            }

        }

        // sekcja przechwytujaca bledy i przygotowujaca do ich rzucania w watku master


        catch(transformer_error::NonFatal_error_1 nfe)
        {
            mt_tt_obj->error_pointer= malloc(sizeof(nfe));
            memcpy(mt_tt_obj->error_pointer, &nfe, sizeof(nfe));
            mt_tt_obj->error_pointer = &nfe;
            mt_tt_obj->error = common::NonFatal_erroR_1;
            mt_tt_obj->trans_t_to_master_order_status_ready();
        } // end: catch(transformer_error::NonFatal_error_1 nfe)

        catch(transformer_error::NonFatal_error_2 nfe)
        {
            mt_tt_obj->error_pointer= malloc(sizeof(nfe));
            memcpy(mt_tt_obj->error_pointer, &nfe, sizeof(nfe));
            mt_tt_obj->error_pointer=&nfe;
            mt_tt_obj->error = common::NonFatal_erroR_2;
            mt_tt_obj->trans_t_to_master_order_status_ready();
        } // end: catch(transformer_error::NonFatal_error_2 nfe)

        catch(transformer_error::NonFatal_error_3 nfe)
        {
            mt_tt_obj->error_pointer= malloc(sizeof(nfe));
            memcpy(mt_tt_obj->error_pointer, &nfe, sizeof(nfe));
            mt_tt_obj->error_pointer=&nfe;
            mt_tt_obj->error = common::NonFatal_erroR_3;
            mt_tt_obj->trans_t_to_master_order_status_ready();
        } // end: catch(transformer_error::NonFatal_error_3 nfe)

        catch(transformer_error::NonFatal_error_4 nfe)
        {
            mt_tt_obj->error_pointer= malloc(sizeof(nfe));
            memcpy(mt_tt_obj->error_pointer, &nfe, sizeof(nfe));
            mt_tt_obj->error = common::NonFatal_erroR_4;
            mt_tt_obj->trans_t_to_master_order_status_ready();
        } // end: catch(transformer_error::NonFatal_error nfe4)

        catch(transformer_error::Fatal_error fe)
        {
            mt_tt_obj->error_pointer= malloc(sizeof(fe));
            memcpy(mt_tt_obj->error_pointer, &fe, sizeof(fe));
            mt_tt_obj->error = common::Fatal_erroR;
            mt_tt_obj->trans_t_to_master_order_status_ready();
        } // end: catch(transformer_error::Fatal_error fe)

        catch (common::System_error fe)
        {
            mt_tt_obj->error_pointer= malloc(sizeof(fe));
            memcpy(mt_tt_obj->error_pointer, &fe, sizeof(fe));
            mt_tt_obj->error = common::System_erroR;
            mt_tt_obj->trans_t_to_master_order_status_ready();
        } // end: catch(System_error fe)

        catch (...)
        {  // Dla zewnetrznej petli try
            printf("transformation thread uneidentified_error\n");

            mt_tt_obj->trans_t_to_master_order_status_ready();
            // Wylapywanie niezdefiniowanych bledow
            // printf("zlapane cos");// by Y&W
        }

    } // end while
    return NULL;
}

} // namespace common
} // namespace edp
} // namespace mrrocpp

