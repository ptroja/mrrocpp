#include "ecp/common/ecp_generator.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

ecp_generator::ecp_generator (common::task::ecp_task& _ecp_task)
        : base(*(ecp_t.sr_ecp_msg)),
        ecp_t(_ecp_task),
        communicate_with_mp_in_move(true),
        communicate_with_edp(true),
        copy_edp_buffers_in_move(true)
{
    the_robot = ecp_t.ecp_m_robot;
    sensor_m.clear();
}

ecp_generator::~ecp_generator()
{}

ecp_generator::ECP_error::ECP_error ( uint64_t err_cl, uint64_t err_no,
                                      uint64_t err0, uint64_t err1 )
        :
        error_class(err_cl),
        error_no(err_no)
{
    error.error0 = err0;
    error.error1 =err1;
}

bool ecp_generator::is_EDP_error (ecp_robot& the_robot) const
{
    // Sprawdzenie czy nie wystapil blad w EDP
    // Funkcja zaklada, ze error_no zostalo zaktualizowane
    // za pomoca conveyor_generator::get_reply
    if ( the_robot.EDP_data.error_no.error0 || the_robot.EDP_data.error_no.error1 )
    {
        return true;
    }
    else
    {
        return false;
    }
}



void ecp_generator::Move()
{
    // Funkcja ruchu dla ECP

    // generacja pierwszego kroku ruchu
    node_counter = 0;
    ecp_t.set_ecp_reply(ECP_ACKNOWLEDGE);

    if (!first_step() ||
            (!(!communicate_with_mp_in_move || ecp_t.mp_buffer_receive_and_send())))
    {
        return; // Warunek koncowy spelniony w pierwszym kroku
    }

    do
    { // realizacja ruchu


        // zadanie przygotowania danych od czujnikow
        ecp_t.all_sensors_initiate_reading(sensor_m);

        // wykonanie kroku ruchu
        if ((the_robot) && communicate_with_edp)
        {
            if (copy_edp_buffers_in_move)
            {
                the_robot->create_command();
            }
            // zlecenie ruchu SET oraz odczyt stanu robota GET

            execute_motion();

            if (copy_edp_buffers_in_move)
            {
                the_robot->get_reply();
            }
        }

        // odczytanie danych z wszystkich czujnikow
        ecp_t.all_sensors_get_reading(sensor_m);
        node_counter++;
        if (ecp_t.pulse_check())
        {
            trigger = true;
        }

    }
    while (next_step() &&
            (!communicate_with_mp_in_move || ecp_t.mp_buffer_receive_and_send()));
}


void ecp_generator::execute_motion(void)
{
	the_robot->execute_motion();
}


} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp


