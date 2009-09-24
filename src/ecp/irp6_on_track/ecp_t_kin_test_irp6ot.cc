#include <time.h>
#include <string.h>
#include <fstream>

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "lib/srlib.h"
#include "ecp_mp/ecp_mp_t_rcsc.h"
#include "ecp_mp/ecp_mp_s_schunk.h"

#include "ecp/irp6_on_track/ecp_r_irp6ot.h"
#include "ecp/common/ecp_g_force.h"
#include "ecp/irp6_on_track/ecp_t_kin_test_irp6ot.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot {
namespace task {

#define BILLION  1000000000L;

// KONSTRUKTORY
kin_test::kin_test(lib::configurator &_config) : task(_config)
{}

// methods for ECP template to redefine in concrete classes
void kin_test::task_initialization(void)
{
    ecp_m_robot = new robot (*this);

    sr_ecp_msg->message("ECP kin test irp6ot loaded");
}


void kin_test::main_task_algorithm(void)
{
    struct timespec start, stop;
    double accum;
    clock_gettime( CLOCK_REALTIME, &start);
    //std::cout<<"Start sec= "<< start.tv_sec <<" nsec= "<< start.tv_nsec << std::endl;
    int n=1000;
    // petla 1000 getow
    for(int i =0; i<n; i++)
    {
        ecp_m_robot->EDP_data.instruction_type = lib::GET;
        ecp_m_robot->EDP_data.get_type = ARM_DV;
        ecp_m_robot->EDP_data.get_arm_type = lib::MOTOR;
        ecp_m_robot->EDP_data.motion_type = lib::ABSOLUTE;
        ecp_m_robot->EDP_data.next_interpolation_type = lib::MIM;
        ecp_m_robot->create_command();
        ecp_m_robot->execute_motion();
    }
    clock_gettime( CLOCK_REALTIME, &stop);

    //std::cout<<"Stop sec= "<< stop.tv_sec <<" nsec= "<< stop.tv_nsec << std::endl;
    //std::cout<<"Diff sec= "<< ( stop.tv_sec - start.tv_sec ) <<" nsec= "<< ( stop.tv_nsec - start.tv_nsec ) << std::endl;

    accum = ( stop.tv_sec - start.tv_sec )
            + (double)( stop.tv_nsec - start.tv_nsec )
            / (double)BILLION;
    //          accum /= n;
    std::cout<<"Time of  get = "<< accum<< "("<<n <<" gets set" <<")"<<std::endl;

    ecp_m_robot->get_reply();

    for(int i =0; i<IRP6_ON_TRACK_NUM_OF_SERVOS; i++)
        std::cout<<ecp_m_robot->EDP_data.current_motor_arm_coordinates[i]<<" ";
    std::cout<<std::endl;

    clock_gettime( CLOCK_REALTIME, &start);
    // petla 100 getow
    for(int i =0; i<n; i++)
    {
        ecp_m_robot->EDP_data.instruction_type = lib::SET;
        ecp_m_robot->EDP_data.set_type = ARM_DV;
        ecp_m_robot->EDP_data.set_arm_type = lib::MOTOR;
        ecp_m_robot->EDP_data.motion_type = lib::ABSOLUTE;
        ecp_m_robot->EDP_data.next_interpolation_type = lib::MIM;
        ecp_m_robot->EDP_data.motion_steps = (uint16_t) 1;
        ecp_m_robot->EDP_data.value_in_step_no = 1;
        memcpy (ecp_m_robot->EDP_data.next_motor_arm_coordinates, ecp_m_robot->EDP_data.current_motor_arm_coordinates, IRP6_ON_TRACK_NUM_OF_SERVOS*sizeof (double));
        ecp_m_robot->create_command();
        ecp_m_robot->execute_motion();

    }
    clock_gettime( CLOCK_REALTIME, &stop);
    accum = ( stop.tv_sec - start.tv_sec )
            + (double)( stop.tv_nsec - start.tv_nsec )
            / (double)BILLION;
    std::cout<<"Time of  set = "<< accum<< "("<<n <<" sets set" <<")"<<std::endl;

    ecp_termination_notice();
}

}
} // namespace irp6ot

namespace common {
namespace task {

task* return_created_ecp_task (lib::configurator &_config)
{
	return new irp6ot::task::kin_test(_config);
}
}
} // namespace common
} // namespace ecp
} // namespace mrrocpp

