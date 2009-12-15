#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "lib/srlib.h"
#include "ecp_mp/task/ecp_mp_t_rcsc.h"
#include "ecp_mp/sensor/ecp_mp_s_schunk.h"

#include "ecp/irp6_on_track/ecp_r_irp6ot.h"
#include "ecp/common/ecp_g_jarosz.h"
#include "ecp/irp6_on_track/ecp_t_pt_irp6ot.h"


namespace mrrocpp {
namespace ecp {
namespace irp6ot {
namespace task {


// KONSTRUKTORY
pteach::pteach(lib::configurator &_config) : task(_config)
{
    ecp_m_robot = new robot (*this);

    cg = new common::generator::calibration (*this, 10);
    // Warunek, ktorego spelnienie umozliwia realizacje ruchu do nastepnej nauczonej pozycji
    orc = new common::operator_reaction_condition (*this);
}


void pteach::main_task_algorithm(void)
{
	if (operator_reaction("Teach in?"))
    {
        cg->flush_pose_list(); // Usuniecie listy pozycji, o ile istnieje
        cg->teach (lib::MOTOR, "Teach-in the trajectory\n");
        if ( operator_reaction ("Save trajectory?") )
        {
            cg->save_file (lib::MOTOR);
        }
    }
    else
        if ( operator_reaction ("Load trajectory?") )
        {
            cg->load_file_from_ui ();
        }

    if ( !operator_reaction ("Start motion?") )
    {
        // Informacja dla MP o zakonczeniu zadania uzytkownika
        ecp_termination_notice ();
        return;
    }

    // Odtwarzanie nauczonej lub wczytanej trajektorii zaczynamy od jej poczatku
    cg->initiate_pose_list();
    // Dlugosc listy = liczba pozycji do odtworzenia
    pll = cg->pose_list_length();
    // Na razie lista pozycji odczytanych we wspolrzednych kartezjanskich jest pusta
    orc->flush_supplementary_list();
    for (i=0; i< pll; i++)
    { // Wewnetrzna petla wykonuje sie pll razy (tyle pozycji nauczono)
        cg->Move();
        orc->Move();
        // oraz oczekiwanie na zezwolenie na kolejny ruch
    } // end: for

    // Czy zapamietac dane?
    if ( operator_reaction ("Save calibration data?") )
    {
        ecp_save_extended_file (*cg, *orc, *this);
    }

    // Informacja dla MP o zakonczeniu zadania uzytkownika
    ecp_termination_notice ();
}

}
} // namespace irp6ot

namespace common {
namespace task {

task* return_created_ecp_task (lib::configurator &_config)
{
	return new irp6ot::task::pteach(_config);
}

}
} // namespace common
} // namespace ecp
} // namespace mrrocpp

