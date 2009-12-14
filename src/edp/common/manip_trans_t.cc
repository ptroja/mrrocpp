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
#include "edp/common/edp_e_manip_and_conv.h"
#include "edp/common/manip_trans_t.h"

/********************************* GLOBALS **********************************/

namespace mrrocpp {
namespace edp {
namespace common {


manip_trans_t::manip_trans_t(manip_and_conv_effector& _master):
	trans_t(_master), master (_master)
{
}

manip_trans_t::~manip_trans_t()
{
}

void manip_trans_t::create_thread(void)
{
	if (pthread_create (&thread_id, NULL, &thread_start, (void *) this))
	{
	    master.msg->message(lib::SYSTEM_ERROR, errno, "EDP: Failed to create manip_trans_t thread");
	    throw System_error();
	}
}


void * manip_trans_t::thread_start(void* arg)
{
    return static_cast<manip_trans_t*> (arg)->thread_main_loop(arg);
}



void * manip_trans_t::thread_main_loop(void *arg)
{

    lib::set_thread_priority(pthread_self(), MAX_PRIORITY);

    while(1)
    {
        // oczekiwanie na zezwolenie ruchu od edp_master
        trans_t_wait_for_master_order();

        // przekopiowanie instrukcji z bufora watku komunikacji z ECP (edp_master)
        master.current_instruction = master.new_instruction;

        error = NO_ERROR; // wyjsciowo brak bledu (dla rzutowania)

        try
        {
			// TODO: this thread is for handling special case of move_arm instruction;
        	// all the othrer call can (and should...) be done from the main communication thread;
        	// they do not need to be processed asynchronously.
            switch (trans_t_task)
            {
            case MT_GET_CONTROLLER_STATE:
            	master.get_controller_state(master.current_instruction);
                trans_t_to_master_order_status_ready();
                break;
            case MT_SET_RMODEL:
            	master.set_rmodel(master.current_instruction);
                trans_t_to_master_order_status_ready();
                break;
            case MT_GET_ARM_POSITION:
            	master.get_arm_position(trans_t_tryb, master.current_instruction);
                trans_t_to_master_order_status_ready();
                break;
            case MT_GET_ALGORITHMS:
            	master.get_algorithms();
                trans_t_to_master_order_status_ready();
                break;
            case MT_SYNCHRONISE:
            	master.synchronise();
                trans_t_to_master_order_status_ready();
                break;
            case MT_MOVE_ARM:
            	master.move_arm(master.current_instruction); 	 // wariant dla watku edp_trans_t
                break;
            default: // blad: z reply_type wynika, e odpowied nie ma zawiera narzedzia
                break;
            }
        }

        // sekcja przechwytujaca bledy i przygotowujaca do ich rzucania w watku master

        catch(transformer_error::NonFatal_error_1 nfe)
        {
            error_pointer= new transformer_error::NonFatal_error_1(nfe);
            error = NonFatal_erroR_1;
            trans_t_to_master_order_status_ready();
        }

        catch(transformer_error::NonFatal_error_2 nfe)
        {
            error_pointer= new transformer_error::NonFatal_error_2(nfe);
            error = NonFatal_erroR_2;
            trans_t_to_master_order_status_ready();
        }

        catch(transformer_error::NonFatal_error_3 nfe)
        {
            error_pointer= new transformer_error::NonFatal_error_3(nfe);
            error = NonFatal_erroR_3;
            trans_t_to_master_order_status_ready();
        }

        catch(transformer_error::NonFatal_error_4 nfe)
        {
            error_pointer= new transformer_error::NonFatal_error_4(nfe);
            error = NonFatal_erroR_4;
            trans_t_to_master_order_status_ready();
        }

        catch(transformer_error::Fatal_error fe)
        {
            error_pointer= new transformer_error::Fatal_error(fe);
            error = Fatal_erroR;
            trans_t_to_master_order_status_ready();
        }

        catch (System_error fe)
        {
            error_pointer= new System_error(fe);
            error = System_erroR;
            trans_t_to_master_order_status_ready();
        }

        catch (...)
        {
            printf("transformation thread unidentified_error\n");

            trans_t_to_master_order_status_ready();
            // Wylapywanie niezdefiniowanych bledow
            // printf("zlapane cos");// by Y&W
        }
    }

    return NULL;
}

} // namespace common
} // namespace edp
} // namespace mrrocpp

