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
#include "edp/common/trans_t.h"

/********************************* GLOBALS **********************************/

namespace mrrocpp {
namespace edp {
namespace common {


trans_t::trans_t(manip_and_conv_effector& _master):
	master (_master)
{
	// semafory do komunikacji miedzy EDP_MASTER a EDP_TRANS
	sem_init(&master_to_trans_t_sem, 0, 0);
	sem_init(&trans_t_to_master_sem, 0, 0);
}

trans_t::~trans_t()
{
	// semafory do komunikacji miedzy EDP_MASTER a EDP_TRANS
	sem_destroy(&master_to_trans_t_sem);
	sem_destroy(&trans_t_to_master_sem);
}

int trans_t::master_to_trans_t_order(MT_ORDER nm_task, int nm_tryb)
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
int trans_t::master_wait_for_trans_t_order_status()
{
	// oczekiwanie na odpowiedz z watku transformation
	return sem_wait(&trans_t_to_master_sem);
}

// oczekiwanie na semafor statusu polecenia z trans_t
int trans_t::trans_t_to_master_order_status_ready()
{
	// odwieszenie watku new master
	sem_trywait(&trans_t_to_master_sem);
	return sem_post(&trans_t_to_master_sem);// odwieszenie watku edp_master
}

// oczekiwanie na semafor statusu polecenia z trans_t
int trans_t::trans_t_wait_for_master_order()
{
	// oczekiwanie na rozkaz z watku master
	return sem_wait(&master_to_trans_t_sem);
}



void * trans_t::trans_thread_start(void* arg)
{
    return static_cast<trans_t*> (arg)->trans_thread(arg);
}

void * trans_t::trans_thread(void *arg)
{

    lib::set_thread_priority(pthread_self(), MAX_PRIORITY);

    while(1)
    {
        // oczekiwanie na zezwolenie ruchu od edp_master
        trans_t_wait_for_master_order();

        // przekopiowanie instrukcji z bufora watku komunikacji z ECP (edp_master)
        memcpy( &(master.current_instruction), &(master.new_instruction), sizeof(lib::c_buffer) );

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

