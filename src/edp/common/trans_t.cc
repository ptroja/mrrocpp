// ------------------------------------------------------------------------
// transformation thread by Y 
// ------------------------------------------------------------------------

/********************************* INCLUDES *********************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "common/typedefs.h"
#include "common/impconst.h"
#include "common/com_buf.h"

#include "lib/mis_fun.h"
#include "edp/common/edp.h"

/********************************* GLOBALS **********************************/

extern edp_irp6s_and_conv_effector* master;   // by Y


void *trans_t_thread(void *arg)
{

	set_thread_priority(pthread_self(), MAX_PRIORITY);
	
	while(1)
	{	
		// oczekiwanie na zezwolenie ruchu od edp_master
		master->mt_tt_obj->trans_t_wait_for_master_order();

		// przekopiowanie instrukcji z bufora watku komunikacji z ECP (edp_master)
		memcpy( &(master->current_instruction), &(master->new_instruction), sizeof(master->new_instruction) ); 
	
		master->mt_tt_obj->error = NO_ERROR; // wyjsciowo brak bledu (dla rzutowania)
	
		try {
		
		switch (master->mt_tt_obj->trans_t_task)
		{
			case MT_GET_CONTROLLER_STATE:
				master->get_controller_state(&(master->current_instruction));
				master->mt_tt_obj->trans_t_to_master_order_status_ready();
				break;
			case MT_SET_RMODEL:
				master->set_rmodel(&(master->current_instruction));
				master->mt_tt_obj->trans_t_to_master_order_status_ready();
				break;
			case MT_GET_ARM_POSITION:
				master->get_arm_position(master->mt_tt_obj->trans_t_tryb, &(master->current_instruction));
				master->mt_tt_obj->trans_t_to_master_order_status_ready();
				break;
			case MT_GET_ALGORITHMS:
				master->get_algorithms();
				master->mt_tt_obj->trans_t_to_master_order_status_ready();
				break;
			case MT_SYNCHRONISE:
				master->synchronise();
				master->mt_tt_obj->trans_t_to_master_order_status_ready();
				break;
			case MT_MOVE_ARM:
				master->move_arm(&(master->current_instruction)); 	 // wariant dla watku edp_trans_t
				break;
			default: // blad: z reply_type wynika, e odpowied nie ma zawiera narzedzia
				break;
		} // end: switch (reply_type)
		
		}
		
		// sekcja przechwytujaca bledy i przygotowujaca do ich rzucania w watku master
		
		catch(transformer_error::NonFatal_error_1 nfe) {
			master->mt_tt_obj->error_pointer= new transformer_error::NonFatal_error_1(nfe);
			master->mt_tt_obj->error = NonFatal_erroR_1;
			master->mt_tt_obj->trans_t_to_master_order_status_ready();
		} // end: catch(transformer::NonFatal_error_1 nfe)
		
		catch(transformer_error::NonFatal_error_2 nfe) {
			master->mt_tt_obj->error_pointer= new transformer_error::NonFatal_error_2(nfe);
			master->mt_tt_obj->error = NonFatal_erroR_2;
			master->mt_tt_obj->trans_t_to_master_order_status_ready();
		} // end: catch(transformer::NonFatal_error_2 nfe)
		
		catch(transformer_error::NonFatal_error_3 nfe) {
			master->mt_tt_obj->error_pointer= new transformer_error::NonFatal_error_3(nfe);
			master->mt_tt_obj->error = NonFatal_erroR_3;
			master->mt_tt_obj->trans_t_to_master_order_status_ready();
		} // end: catch(transformer::NonFatal_error_3 nfe)
		
		catch(transformer_error::NonFatal_error_4 nfe) {
			master->mt_tt_obj->error_pointer= new transformer_error::NonFatal_error_4(nfe);
			master->mt_tt_obj->error = NonFatal_erroR_4;
			master->mt_tt_obj->trans_t_to_master_order_status_ready();
		} // end: catch(transformer::NonFatal_error nfe4)
		
		catch(transformer_error::Fatal_error fe) {
			master->mt_tt_obj->error_pointer= new transformer_error::Fatal_error(fe);
			master->mt_tt_obj->error = Fatal_erroR;
			master->mt_tt_obj->trans_t_to_master_order_status_ready();
		} // end: catch(transformer::Fatal_error fe)
				
		catch (System_error fe) {
			master->mt_tt_obj->error_pointer= new System_error(fe);
			master->mt_tt_obj->error = System_erroR;
			master->mt_tt_obj->trans_t_to_master_order_status_ready();
		} // end: catch(System_error fe)
		
		catch (...) {   
			printf("transformation thread unidentified_error\n");
		
			master->mt_tt_obj->trans_t_to_master_order_status_ready();
			// Wylapywanie niezdefiniowanych bledow
			// printf("zlapane cos");// by Y&W
		}
		
	} // end  while
}
