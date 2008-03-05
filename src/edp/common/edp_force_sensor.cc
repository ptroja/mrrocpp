// ------------------------------------------------------------------------
//                                  edp.cc
//
// EDP_MASTER Effector Driver Master Process
// Driver dla robota IRp-6 na torze - metody: class edp_irp6s_robot
//
// Ostatnia modyfikacja: styczen 2005
// -------------------------------------------------------------------------

#include <edp/common/edp.h>
#include "edp/common/edp_irp6s_postument_track.h"

edp_force_sensor::edp_force_sensor(edp_irp6s_postument_track_effector &_master)
        : master(_master), new_edp_command(false)
{
    gravity_transformation = NULL;
    is_sensor_configured=false;	//!< czujnik niezainicjowany
    is_reading_ready=false;				//!< nie ma zadnego gotowego odczytu
    force_sensor_do_configure = false;
    force_sensor_set_tool = false;

    sem_init( &new_ms, 0, 0);
    sem_init( &new_ms_for_edp, 0, 0);
};

void edp_force_sensor::wait_for_event(void)
{}
;			// oczekiwanie na zdarzenie

void edp_force_sensor::set_force_tool(void)
{

    K_vector gravity_arm_in_sensor(next_force_tool_position);
    Homog_matrix frame = master.return_current_frame(WITH_TRANSLATION);
    gravity_transformation->defineTool(frame, next_force_tool_weight, gravity_arm_in_sensor);

    for (int i = 0; i<3; i++)
    {
        current_force_tool_position[i] = next_force_tool_position[i];
    }
    current_force_tool_weight = next_force_tool_weight;

}
;

int	edp_force_sensor::set_command_execution_finish() // podniesienie semafora
{
    if (new_edp_command)
    {
        new_edp_command = false;
        sem_trywait(&(new_ms_for_edp));
        return sem_post(&new_ms_for_edp);// odwieszenie watku edp_master
    }

};

int	edp_force_sensor::check_for_command_execution_finish() // oczekiwanie na semafor
{
    new_edp_command = true;
    return sem_wait(&new_ms_for_edp);
};
