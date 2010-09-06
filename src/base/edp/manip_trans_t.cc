// ------------------------------------------------------------------------
// transformation thread by Y
// ------------------------------------------------------------------------

/********************************* INCLUDES *********************************/
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"

#include "base/lib/mis_fun.h"
#include "base/edp/edp_e_motor_driven.h"
#include "base/edp/manip_trans_t.h"

/********************************* GLOBALS **********************************/

namespace mrrocpp {
namespace edp {
namespace common {


manip_trans_t::manip_trans_t(motor_driven_effector& _master):
	trans_t(_master), master (_master)
{
	thread_id = new boost::thread(boost::bind(&manip_trans_t::operator(), this));
}

manip_trans_t::~manip_trans_t()
{
	delete thread_id;
}

void manip_trans_t::operator()()
{
    lib::set_thread_priority(pthread_self(), lib::QNX_MAX_PRIORITY);

    while(1)
    {
        // oczekiwanie na zezwolenie ruchu od edp_master
    	master_to_trans_synchroniser.wait();

        // przekopiowanie instrukcji z bufora watku komunikacji z ECP (edp_master)
//        master.current_instruction = master.instruction;

        error = NO_ERROR; // wyjsciowo brak bledu (dla rzutowania)

        try
        {
			// TODO: this thread is for handling special case of move_arm instruction;
        	// all the othrer call can (and should...) be done from the main communication thread;
        	// they do not need to be processed asynchronously.
            switch (trans_t_task)
            {
            case MT_GET_CONTROLLER_STATE:
            	master.get_controller_state(instruction);
                trans_t_to_master_synchroniser.command();
                break;
            case MT_SET_ROBOT_MODEL:
            	master.set_robot_model(instruction);
                trans_t_to_master_synchroniser.command();
                break;
            case MT_GET_ARM_POSITION:
            	master.get_arm_position(trans_t_tryb, instruction);
                trans_t_to_master_synchroniser.command();
                break;
            case MT_GET_ALGORITHMS:
            	master.get_algorithms();
                trans_t_to_master_synchroniser.command();
                break;
            case MT_SYNCHRONISE:
            	master.synchronise();
                trans_t_to_master_synchroniser.command();
                break;
            case MT_MOVE_ARM:
            	master.move_arm(instruction); 	 // wariant dla watku edp_trans_t
                break;
            default: // blad: z reply_type wynika, e odpowied nie ma zawiera narzedzia
                break;
            }
        }

        // sekcja przechwytujaca bledy i przygotowujaca do ich rzucania w watku master

        catch(NonFatal_error_1 nfe)
        {
            error_pointer= new NonFatal_error_1(nfe);
            error = NonFatal_erroR_1;
            trans_t_to_master_synchroniser.command();
        }

        catch(NonFatal_error_2 nfe)
        {
            error_pointer= new NonFatal_error_2(nfe);
            error = NonFatal_erroR_2;
            trans_t_to_master_synchroniser.command();
        }

        catch(NonFatal_error_3 nfe)
        {
            error_pointer= new NonFatal_error_3(nfe);
            error = NonFatal_erroR_3;
            trans_t_to_master_synchroniser.command();
        }

        catch(NonFatal_error_4 nfe)
        {
            error_pointer= new NonFatal_error_4(nfe);
            error = NonFatal_erroR_4;
            trans_t_to_master_synchroniser.command();
        }

        catch(Fatal_error fe)
        {
            error_pointer= new Fatal_error(fe);
            error = Fatal_erroR;
            trans_t_to_master_synchroniser.command();
        }

        catch (System_error fe)
        {
            error_pointer= new System_error(fe);
            error = System_erroR;
            trans_t_to_master_synchroniser.command();
        }

        catch (...)
        {
            printf("transformation thread unidentified_error\n");

            trans_t_to_master_synchroniser.command();
            // Wylapywanie niezdefiniowanych bledow
            // printf("zlapane cos");// by Y&W
        }
    }
}

} // namespace common
} // namespace edp
} // namespace mrrocpp

