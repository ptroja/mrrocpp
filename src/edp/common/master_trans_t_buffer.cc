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
#include "edp/common/edp_e_manip.h"
#include "edp/common/master_trans_t_buffer.h"

/********************************* GLOBALS **********************************/

namespace mrrocpp {
namespace edp {
namespace common {

int	manip_effector::master_order(MT_ORDER nm_task, int nm_tryb)
{
	return manip_and_conv_effector::master_order(nm_task, nm_tryb);
}


master_trans_t_buffer::master_trans_t_buffer()
{
	// semafory do komunikacji miedzy EDP_MASTER a EDP_TRANS
	sem_init(&master_to_trans_t_sem, 0, 0);
	sem_init(&trans_t_to_master_sem, 0, 0);
}

master_trans_t_buffer::~master_trans_t_buffer()
{
	// semafory do komunikacji miedzy EDP_MASTER a EDP_TRANS
	sem_destroy(&master_to_trans_t_sem);
	sem_destroy(&trans_t_to_master_sem);
}

int master_trans_t_buffer::master_to_trans_t_order(MT_ORDER nm_task, int nm_tryb)
{ // zlecenie z watku master dla trans_t
	trans_t_task = nm_task; // force, arm etc.
	trans_t_tryb = nm_tryb; // tryb dla zadania

	// odwieszenie watku transformation
	sem_trywait(&master_to_trans_t_sem);
	sem_post(&master_to_trans_t_sem);

	// oczekiwanie na zwolniene samafora przez watek trans_t
	sem_wait(&trans_t_to_master_sem); // oczekiwanie na zezwolenie ruchu od edp_master

	// sekcja sprawdzajaca czy byl blad w watku transforamation i ew. rzucajaca go w watku master

	switch (error) {
		case NonFatal_erroR_1:
			throw *(kinematic::common::transformer_error::NonFatal_error_1*)(error_pointer);
			break;
		case NonFatal_erroR_2:
			throw *(kinematic::common::transformer_error::NonFatal_error_2*)(error_pointer);
			break;
		case NonFatal_erroR_3:
			throw *(kinematic::common::transformer_error::NonFatal_error_3*)(error_pointer);
			break;
		case NonFatal_erroR_4:
			throw *(kinematic::common::transformer_error::NonFatal_error_4*)(error_pointer);
			break;
		case Fatal_erroR:
			throw *(kinematic::common::transformer_error::Fatal_error*)(error_pointer);
			break;
		case System_erroR:
			throw *(System_error*)(error_pointer);
			break;
		default:
			break;
	}

	return 1;
}

// oczekiwanie na semafor statusu polecenia z trans_t
int master_trans_t_buffer::master_wait_for_trans_t_order_status()
{
	// oczekiwanie na odpowiedz z watku transformation
	return sem_wait(&trans_t_to_master_sem);
}

// oczekiwanie na semafor statusu polecenia z trans_t
int master_trans_t_buffer::trans_t_to_master_order_status_ready()
{
	// odwieszenie watku new master
	sem_trywait(&trans_t_to_master_sem);
	return sem_post(&trans_t_to_master_sem);// odwieszenie watku edp_master
}

// oczekiwanie na semafor statusu polecenia z trans_t
int master_trans_t_buffer::trans_t_wait_for_master_order()
{
	// oczekiwanie na rozkaz z watku master
	return sem_wait(&master_to_trans_t_sem);
}



void * manip_and_conv_effector::trans_thread_start(void* arg)
{
    return static_cast<manip_and_conv_effector*> (arg)->trans_thread(arg);
}

void * manip_and_conv_effector::trans_thread(void *arg)
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
			// TODO: this thread is for handling special case of move_arm instruction;
        	// all the othrer call can (and should...) be done from the main communication thread;
        	// they do not need to be processed asynchronously.
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

    return NULL;
}

} // namespace common
} // namespace edp
} // namespace mrrocpp

