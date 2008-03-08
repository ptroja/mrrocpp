#include <stdio.h>

#include "common/typedefs.h"
#include "common/impconst.h"
#include "common/com_buf.h"

#include "lib/srlib.h"
#include "ecp_mp/ecp_mp_t_rcsc.h"
#include "ecp_mp/ecp_mp_s_schunk.h"

#include "ecp/irp6_on_track/ecp_local.h"
#include "ecp/common/ecp_g_force.h"
#include "ecp/common/ecp_g_smooth.h"
#include "ecp/common/ecp_t_rcsc.h"
#include "ecp/irp6_on_track/ecp_t_rcsc_irp6ot.h"

ecp_task_rcsc_irp6ot::ecp_task_rcsc_irp6ot(configurator &_config) :
        ecp_task(_config)
{}

ecp_task_rcsc_irp6ot::~ecp_task_rcsc_irp6ot()
{}

// methods for ECP template to redefine in concrete classes
void ecp_task_rcsc_irp6ot::task_initialization(void)
{
    // the robot is choose dependendat on the section of configuration file sent as argv[4]
    ecp_m_robot = new ecp_irp6_on_track_robot (*this);


    gt = new ecp_generator_t (*this);

    nrg = new ecp_tff_nose_run_generator (*this, 8);
    //	nrg->configure(false, false, true, false, false, false, false);

    rgg = new ecp_tff_rubik_grab_generator (*this, 8);

    gag = new ecp_tff_gripper_approach_generator (*this, 8);

    rfrg = new ecp_tff_rubik_face_rotate_generator (*this, 8);

    tig = new ecp_teach_in_generator (*this);

    befg = new bias_edp_force_generator (*this);

    sg = new ecp_smooth_generator (*this, true);

    sr_ecp_msg->message("ECP loaded");
}

void ecp_task_rcsc_irp6ot::main_task_algorithm(void)
{

    int size;
    char * path1;

    sr_ecp_msg->message("ECP rcsc irp6ot  - wcisnij start");
    ecp_wait_for_start();
    for(;;)
    { // Wewnetrzna petla nieskonczona

        for(;;)
        {
            sr_ecp_msg->message("Waiting for MP order");

            get_next_state ();

            sr_ecp_msg->message("Order received");

            switch ( (RCSC_ECP_STATES) mp_command.mp_package.ecp_next_state.mp_2_ecp_next_state)
            {
            case ECP_GEN_TRANSPARENT:
                Move (*gt);
                break;
            case ECP_GEN_BIAS_EDP_FORCE:
                Move (*befg);
                break;
            case ECP_GEN_TFF_NOSE_RUN:
                Move (*nrg);
                break;
            case ECP_GEN_TFF_RUBIK_GRAB:
                switch ( (RCSC_RUBIK_GRAB_PHASES) mp_command.mp_package.ecp_next_state.mp_2_ecp_next_state_variant)
                {
                case RCSC_RG_FROM_OPEARTOR_PHASE_1:
                    rgg->configure(0.057, 0.00005, 0);
                    break;
                case RCSC_RG_FROM_OPEARTOR_PHASE_2:
                    rgg->configure(0.057, 0.00005, 50);
                    break;
                case RCSC_RG_FCHANGE_PHASE_1:
                    rgg->configure(0.072, 0.00005, 0, false);
                    break;
                case RCSC_RG_FCHANGE_PHASE_2:
                    rgg->configure(0.062, 0.00005, 0);
                    break;
                case RCSC_RG_FCHANGE_PHASE_3:
                    rgg->configure(0.057, 0.00005, 0);
                    break;
                case RCSC_RG_FCHANGE_PHASE_4:
                    rgg->configure(0.057, 0.00005, 50);
                    break;
                default:
                    break;
                }
                Move (*rgg);
                break;
            case ECP_GEN_TFF_GRIPPER_APPROACH:
                gag->configure(5.0, 150);
                Move (*gag);
                break;
            case ECP_GEN_TFF_RUBIK_FACE_ROTATE:
                switch ( (RCSC_TURN_ANGLES) mp_command.mp_package.ecp_next_state.mp_2_ecp_next_state_variant)
                {
                case RCSC_CCL_90:
                    rfrg->configure(-90.0);
                    break;
                case RCSC_CL_0:
                    rfrg->configure(0.0);
                    break;
                case RCSC_CL_90:
                    rfrg->configure(90.0);
                    break;
                case RCSC_CL_180:
                    rfrg->configure(180.0);
                    break;
                default:
                    break;
                }
                Move (*rfrg);
                break;
            case RCSC_GRIPPER_OPENING:
                switch ( (RCSC_TURN_ANGLES) mp_command.mp_package.ecp_next_state.mp_2_ecp_next_state_variant)
                {
                case RCSC_GO_VAR_1:
                    ecp_gripper_opening ( *this, 0.002, 1000);
                    break;
                case RCSC_GO_VAR_2:
                    ecp_gripper_opening ( *this, 0.02, 1000);
                    break;
                default:
                    break;
                }
                break;
            case ECP_GEN_TEACH_IN:
                size = 1 + strlen(mrrocpp_network_path) + strlen(mp_command.mp_package.ecp_next_state.mp_2_ecp_next_state_string);
                path1 = new char[size];
                // Stworzenie sciezki do pliku.
                strcpy(path1, mrrocpp_network_path);
                sprintf(path1, "%s%s", mrrocpp_network_path, mp_command.mp_package.ecp_next_state.mp_2_ecp_next_state_string);
                tig->flush_pose_list();
                tig->load_file_with_path (path1);
                //		printf("\nTRACK ECP_GEN_TEACH_IN :%s\n\n", path1);
                tig->initiate_pose_list();
                delete[] path1;
                Move (*tig);
                break;
            case ECP_GEN_SMOOTH:
                size = 1 + strlen(mrrocpp_network_path) + strlen(mp_command.mp_package.ecp_next_state.mp_2_ecp_next_state_string);
                path1 = new char[size];
                // Stworzenie sciezki do pliku.
                strcpy(path1, mrrocpp_network_path);
                sprintf(path1, "%s%s", mrrocpp_network_path, mp_command.mp_package.ecp_next_state.mp_2_ecp_next_state_string);
                sg->load_file_with_path (path1);
                //				printf("\nTRACK ECP_GEN_SMOOTH :%s\n\n", path1);
                delete[] path1;
                Move (*sg);
                break;
            default:
                break;
            }
			ecp_termination_notice();

        } //end for

        // Oczekiwanie na STOP
        printf("przed wait for stop\n");
        ecp_wait_for_stop ();
        break;
    } // koniec: for(;;) wewnetrznej

}

ecp_task* return_created_ecp_task(configurator &_config)
                {
                    return new ecp_task_rcsc_irp6ot(_config);
                }
