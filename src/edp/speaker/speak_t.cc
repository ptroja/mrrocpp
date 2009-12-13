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
#include "edp/speaker/speak_t.h"

namespace mrrocpp {
namespace edp {
namespace speaker {

speak_t::speak_t(effector& _master):
	trans_t(_master), master (_master)
{
}

speak_t::~speak_t()
{
}

void speak_t::create_thread(void)
{
	if (pthread_create (&thread_id, NULL, &thread_start, (void *) this))
	{
	    master.msg->message(lib::SYSTEM_ERROR, errno, "EDP: Failed to create speak_t thread");
	    throw common::System_error();
	}
}

void * speak_t::thread_start(void* arg)
{
//	 edp_irp6s_and_conv_effector *master = (edp_irp6s_and_conv_effector *) arg;

	 return static_cast<speak_t*> (arg)->thread_main_loop(arg);
}


void * speak_t::thread_main_loop(void *arg)
{
    lib::set_thread_priority(pthread_self() , MAX_PRIORITY-10);

    if( master.init() == -1)
    {
        master.initialize_incorrect=1;
    }
    else
    {
    	master.initialize_incorrect=0;
    }

    while(1)
    {
        trans_t_wait_for_master_order();// oczekiwanie na zezwolenie ruchu od edp_master

        // przekopiowanie instrukcji z bufora watku komunikacji z ECP (edp_master)
        memcpy( &(master.current_instruction), &(master.new_instruction), sizeof(master.new_instruction) );
        try
        {

            switch (trans_t_task)
            {
            case common::MT_GET_ARM_POSITION:
                // master.get_arm_position(trans_t_tryb, &(master.current_instruction));
            	master.get_spoken(trans_t_tryb, &(master.current_instruction)); // MAC7
                trans_t_to_master_order_status_ready();
                break;
            case common::MT_MOVE_ARM:
                // master.move_arm(&(master.current_instruction)); 	 // wariant dla watku edp_trans_t
                trans_t_to_master_order_status_ready();
                master.speak(&(master.current_instruction)); // MAC7
                break;
            default: // blad: z reply_type wynika, e odpowied nie ma zawiera narzedzia
                break;
            }

        }

        // sekcja przechwytujaca bledy i przygotowujaca do ich rzucania w watku master


        catch(transformer_error::NonFatal_error_1 nfe)
        {
            error_pointer= malloc(sizeof(nfe));
            memcpy(error_pointer, &nfe, sizeof(nfe));
            error_pointer = &nfe;
            error = common::NonFatal_erroR_1;
            trans_t_to_master_order_status_ready();
        } // end: catch(transformer_error::NonFatal_error_1 nfe)

        catch(transformer_error::NonFatal_error_2 nfe)
        {
            error_pointer= malloc(sizeof(nfe));
            memcpy(error_pointer, &nfe, sizeof(nfe));
            error_pointer=&nfe;
            error = common::NonFatal_erroR_2;
            trans_t_to_master_order_status_ready();
        } // end: catch(transformer_error::NonFatal_error_2 nfe)

        catch(transformer_error::NonFatal_error_3 nfe)
        {
            error_pointer= malloc(sizeof(nfe));
            memcpy(error_pointer, &nfe, sizeof(nfe));
            error_pointer=&nfe;
            error = common::NonFatal_erroR_3;
            trans_t_to_master_order_status_ready();
        } // end: catch(transformer_error::NonFatal_error_3 nfe)

        catch(transformer_error::NonFatal_error_4 nfe)
        {
            error_pointer= malloc(sizeof(nfe));
            memcpy(error_pointer, &nfe, sizeof(nfe));
            error = common::NonFatal_erroR_4;
            trans_t_to_master_order_status_ready();
        } // end: catch(transformer_error::NonFatal_error nfe4)

        catch(transformer_error::Fatal_error fe)
        {
            error_pointer= malloc(sizeof(fe));
            memcpy(error_pointer, &fe, sizeof(fe));
            error = common::Fatal_erroR;
            trans_t_to_master_order_status_ready();
        } // end: catch(transformer_error::Fatal_error fe)

        catch (common::System_error fe)
        {
            error_pointer= malloc(sizeof(fe));
            memcpy(error_pointer, &fe, sizeof(fe));
            error = common::System_erroR;
            trans_t_to_master_order_status_ready();
        } // end: catch(System_error fe)

        catch (...)
        {  // Dla zewnetrznej petli try
            printf("transformation thread uneidentified_error\n");

            trans_t_to_master_order_status_ready();
            // Wylapywanie niezdefiniowanych bledow
            // printf("zlapane cos");// by Y&W
        }

    } // end while
    return NULL;
}

} // namespace common
} // namespace edp
} // namespace mrrocpp

