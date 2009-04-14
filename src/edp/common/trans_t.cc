// ------------------------------------------------------------------------
// transformation thread by Y
// ------------------------------------------------------------------------

/********************************* INCLUDES *********************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "lib/mis_fun.h"
#include "edp/common/edp.h"

/********************************* GLOBALS **********************************/

namespace mrrocpp {
namespace edp {
namespace common {

void * irp6s_and_conv_effector::trans_thread_start(void* arg)
{
    static_cast<irp6s_and_conv_effector*> (arg)->trans_thread(arg);
}



void * irp6s_and_conv_effector::trans_thread(void *arg)
{

    lib::set_thread_priority(pthread_self(), MAX_PRIORITY);

    while(1)
    {
        // oczekiwanie na zezwolenie ruchu od edp_master
        mt_tt_obj->trans_t_wait_for_master_order();

        // przekopiowanie instrukcji z bufora watku komunikacji z ECP (edp_master)
        memcpy( &(current_instruction), &(new_instruction), sizeof(lib::c_buffer) );

        mt_tt_obj->error = NO_ERROR; // wyjsciowo brak bledu (dla rzutowania)

        try
        {

            switch (mt_tt_obj->trans_t_task)
            {
            case MT_GET_CONTROLLER_STATE:
                get_controller_state(current_instruction);
                mt_tt_obj->trans_t_to_master_order_status_ready();
                break;
            case MT_SET_RMODEL:
                set_rmodel(current_instruction);
                mt_tt_obj->trans_t_to_master_order_status_ready();
                break;
            case MT_GET_ARM_POSITION:
                get_arm_position(mt_tt_obj->trans_t_tryb, current_instruction);
                mt_tt_obj->trans_t_to_master_order_status_ready();
                break;
            case MT_GET_ALGORITHMS:
                get_algorithms();
                mt_tt_obj->trans_t_to_master_order_status_ready();
                break;
            case MT_SYNCHRONISE:
                synchronise();
                mt_tt_obj->trans_t_to_master_order_status_ready();
                break;
            case MT_MOVE_ARM:
                move_arm(current_instruction); 	 // wariant dla watku edp_trans_t
                break;
            default: // blad: z reply_type wynika, e odpowied nie ma zawiera narzedzia
                break;
            }
        }

        // sekcja przechwytujaca bledy i przygotowujaca do ich rzucania w watku master

        catch(transformer_error::NonFatal_error_1 nfe)
        {
            mt_tt_obj->error_pointer= new transformer_error::NonFatal_error_1(nfe);
            mt_tt_obj->error = NonFatal_erroR_1;
            mt_tt_obj->trans_t_to_master_order_status_ready();
        }

        catch(transformer_error::NonFatal_error_2 nfe)
        {
            mt_tt_obj->error_pointer= new transformer_error::NonFatal_error_2(nfe);
            mt_tt_obj->error = NonFatal_erroR_2;
            mt_tt_obj->trans_t_to_master_order_status_ready();
        }

        catch(transformer_error::NonFatal_error_3 nfe)
        {
            mt_tt_obj->error_pointer= new transformer_error::NonFatal_error_3(nfe);
            mt_tt_obj->error = NonFatal_erroR_3;
            mt_tt_obj->trans_t_to_master_order_status_ready();
        }

        catch(transformer_error::NonFatal_error_4 nfe)
        {
            mt_tt_obj->error_pointer= new transformer_error::NonFatal_error_4(nfe);
            mt_tt_obj->error = NonFatal_erroR_4;
            mt_tt_obj->trans_t_to_master_order_status_ready();
        }

        catch(transformer_error::Fatal_error fe)
        {
            mt_tt_obj->error_pointer= new transformer_error::Fatal_error(fe);
            mt_tt_obj->error = Fatal_erroR;
            mt_tt_obj->trans_t_to_master_order_status_ready();
        }

        catch (System_error fe)
        {
            mt_tt_obj->error_pointer= new System_error(fe);
            mt_tt_obj->error = System_erroR;
            mt_tt_obj->trans_t_to_master_order_status_ready();
        }

        catch (...)
        {
            printf("transformation thread unidentified_error\n");

            mt_tt_obj->trans_t_to_master_order_status_ready();
            // Wylapywanie niezdefiniowanych bledow
            // printf("zlapane cos");// by Y&W
        }
    }
}

} // namespace common
} // namespace edp
} // namespace mrrocpp

