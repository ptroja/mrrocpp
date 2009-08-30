// ------------------------------------------------------------------------
//                                  edp.cc
//
// EDP_MASTER Effector Driver Master Process
// Driver dla robota IRp-6 na torze - metody: class edp_irp6s_robot
//
// Ostatnia modyfikacja: styczen 2005
// -------------------------------------------------------------------------

#include <stdio.h>
#include <semaphore.h>

#include "edp/common/edp.h"
#include "edp/common/edp_irp6s_postument_track.h"

namespace mrrocpp {
namespace edp {
namespace sensor {

force::force(common::irp6s_postument_track_effector &_master)
        : new_edp_command(false), master(_master)
{
    gravity_transformation = NULL;
    is_sensor_configured=false;	//!< czujnik niezainicjowany
    first_configure_done=false;
    is_reading_ready=false;				//!< nie ma zadnego gotowego odczytu
    force_sensor_do_first_configure = false;
    force_sensor_do_configure = false;
    force_sensor_set_tool = false;
    TERMINATE = false;

    sem_init( &new_ms, 0, 0);
    sem_init( &new_ms_for_edp, 0, 0);

    /*!Lokalizacja procesu wywietlania komunikatow SR */
    sr_msg = new lib::sr_vsp(lib::EDP, master.config.return_attach_point_name(lib::configurator::CONFIG_SERVER, "edp_vsp_attach_point").c_str(),
                                 master.config.return_attach_point_name(lib::configurator::CONFIG_SERVER, "sr_attach_point", "[ui]").c_str());
}

// oczekiwanie na zdarzenie
void force::wait_for_event(void)
{}

void force::set_force_tool(void)
{

    lib::K_vector gravity_arm_in_sensor(next_force_tool_position);
    lib::Homog_matrix frame = master.return_current_frame(common::WITH_TRANSLATION);
    gravity_transformation->defineTool(frame, next_force_tool_weight, gravity_arm_in_sensor);

    for (int i = 0; i<3; i++)
    {
        current_force_tool_position[i] = next_force_tool_position[i];
    }
    current_force_tool_weight = next_force_tool_weight;
}

int	force::set_command_execution_finish() // podniesienie semafora
{
    if (new_edp_command)
    {
        new_edp_command = false;
        sem_trywait(&(new_ms_for_edp));
        return sem_post(&new_ms_for_edp);// odwieszenie watku edp_master
    }

    return 0; // TODO: check for return or throw in future object-oriented version
}

int	force::check_for_command_execution_finish() // oczekiwanie na semafor
{
    new_edp_command = true;
    return sem_wait(&new_ms_for_edp);
}

} // namespace sensor
} // namespace edp
} // namespace mrrocpp

