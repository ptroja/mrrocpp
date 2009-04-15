// -------------------------------------------------------------------------
//                              mp.cc
//
// MP Master Process - methods
//
// -------------------------------------------------------------------------
// Funkcje do konstruowania procesow MP

#include <unistd.h>
#include <string.h>

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "lib/srlib.h"
#include "mp/mp.h"

#include "ecp_mp/ecp_mp_t_player.h"

using namespace std;

namespace mrrocpp {
namespace mp {
namespace common {




mp_taught_in_pose::mp_taught_in_pose(void)
{}

mp_taught_in_pose::mp_taught_in_pose(lib::POSE_SPECIFICATION at, double mt, double* c) :
        arm_type(at), motion_time(mt)
{
    memcpy(coordinates, c, MAX_SERVOS_NR*sizeof(double));
}

mp_taught_in_pose::mp_taught_in_pose(lib::POSE_SPECIFICATION at, double mt,
                                     double* c, double* irp6p_c) :
        arm_type(at), motion_time(mt)
{
    memcpy(coordinates, c, MAX_SERVOS_NR*sizeof(double));
    memcpy(irp6p_coordinates, irp6p_c, MAX_SERVOS_NR*sizeof(double));
}

mp_taught_in_pose::mp_taught_in_pose(lib::POSE_SPECIFICATION at, double mt,
                                     int e_info, double* c) :
        arm_type(at), motion_time(mt)
{ // by Y
    memcpy(coordinates, c, MAX_SERVOS_NR*sizeof(double));
    extra_info = e_info;
}

robot::MP_error::MP_error(lib::ERROR_CLASS err0, uint64_t err1) :
        error_class(err0), mp_error(err1)
{}

// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// ##############################################################
// ##############################################################
//                              CIALA METOD dla generatorow
// ##############################################################
// ##############################################################
// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

} // namespace common
} // namespace mp
} // namespace mrrocpp

#include "mp/mp_common_generators.h"


namespace mrrocpp {
namespace mp {
namespace generator {


// generator for setting the next ecps state

set_next_ecps_state::set_next_ecps_state(task::base& _mp_task):
        generator (_mp_task)
{}

void set_next_ecps_state::configure (int l_mp_2_ecp_next_state, int l_mp_2_ecp_next_state_variant,
        const char* l_mp_2_ecp_next_state_string)
{
    ecp_next_state.mp_2_ecp_next_state = l_mp_2_ecp_next_state;
    ecp_next_state.mp_2_ecp_next_state_variant = l_mp_2_ecp_next_state_variant;
    if (l_mp_2_ecp_next_state_string)
    {
        strcpy (ecp_next_state.mp_2_ecp_next_state_string, l_mp_2_ecp_next_state_string);
    }
}

void set_next_ecps_state::configure (const lib::playerpos_goal_t &_goal)
{
    ecp_next_state.mp_2_ecp_next_state = ecp_mp::task::ECP_GEN_PLAYERPOS;
    ecp_next_state.playerpos_goal = _goal;
}

// ----------------------------------------------------------------------------------------------
// ---------------------------------    metoda	first_step -------------------------------------
// ----------------------------------------------------------------------------------------------

bool set_next_ecps_state::first_step ()
{
    for (map <lib::ROBOT_ENUM, common::robot*>::iterator robot_m_iterator = robot_m.begin();
            robot_m_iterator != robot_m.end(); robot_m_iterator++)
    {

        robot_m_iterator->second->ecp_td.mp_command = lib::NEXT_STATE;

        robot_m_iterator->second->ecp_td.ecp_next_state = ecp_next_state;

        robot_m_iterator->second->communicate = true;
    }

    return true;
}

// ----------------------------------------------------------------------------------------------
// -----------------------------------  metoda	next_step -----------------------------------
// ----------------------------------------------------------------------------------------------

bool set_next_ecps_state::next_step ()
{
    return false;
}

send_end_motion_to_ecps::send_end_motion_to_ecps(task::base& _mp_task)
        : generator (_mp_task)
{}

// ----------------------------------------------------------------------------------------------
// ---------------------------------    metoda	first_step -------------------------------------
// ----------------------------------------------------------------------------------------------

bool send_end_motion_to_ecps::first_step ()
{
    for (map <lib::ROBOT_ENUM, common::robot*>::iterator robot_m_iterator = robot_m.begin();
            robot_m_iterator != robot_m.end(); robot_m_iterator++)
    {
        robot_m_iterator->second->ecp_td.mp_command = lib::END_MOTION;
        robot_m_iterator->second->communicate = true;
    }

    return true;
}

// ----------------------------------------------------------------------------------------------
// -----------------------------------  metoda	next_step -----------------------------------
// ----------------------------------------------------------------------------------------------

bool send_end_motion_to_ecps::next_step ()
{
    return false;
}

// ###############################################################
// Rozszerzony generator pusty. Faktyczna generacja trajektorii odbywa sie w ECP
// ###############################################################

extended_empty::extended_empty(task::base& _mp_task):
	generator (_mp_task)
{
    activate_trigger = true;
}

void extended_empty::configure (bool l_activate_trigger)
{
    activate_trigger = l_activate_trigger;
}

// ----------------------------------------------------------------------------------------------
// ---------------------------------    metoda	first_step -------------------------------------
// ----------------------------------------------------------------------------------------------

bool extended_empty::first_step ()
{

    wait_for_ECP_pulse = true;
    for (map <lib::ROBOT_ENUM, common::robot*>::iterator robot_m_iterator = robot_m.begin();
            robot_m_iterator != robot_m.end(); robot_m_iterator++)
    {
        robot_m_iterator->second->ecp_td.mp_command = lib::NEXT_POSE;
        robot_m_iterator->second->ecp_td.instruction_type = lib::QUERY;
        robot_m_iterator->second->communicate = false;
    }

    return true;
}

// ----------------------------------------------------------------------------------------------
// -----------------------------------  metoda	next_step --------------------------------------
// ----------------------------------------------------------------------------------------------

bool extended_empty::next_step ()
{
    // Funkcja zwraca false gdy koniec generacji trajektorii
    // Funkcja zwraca true gdy generacja trajektorii bedzie kontynuowana
    // Na podstawie ecp_reply dla poszczegolnych robotow nalezy okreslic czy
    // skonczono zadanie uzytkownika



    // 	if (trigger) printf("Yh\n"); else printf("N\n");
    // printf("mp next step\n");
    // UWAGA: dzialamy na jednoelementowej liscie robotow

    if (check_and_null_trigger() && activate_trigger)
    {
        return false;
    }

    for (map <lib::ROBOT_ENUM, common::robot*>::iterator robot_m_iterator = robot_m.begin();
            robot_m_iterator != robot_m.end(); robot_m_iterator++)
    {
        if (robot_m_iterator->second->new_pulse)
        {
            // printf("mpextempty_gen r: %d, pc: %d\n", robot_m_iterator->first, robot_m_iterator->second->pulse_code);
            robot_m_iterator->second->communicate = true;
        }
        else
        {
            robot_m_iterator->second->communicate = false;
        }
    }

    for (map <lib::ROBOT_ENUM, common::robot*>::iterator robot_m_iterator = robot_m.begin();
            robot_m_iterator != robot_m.end(); robot_m_iterator++)
    {
        if ( robot_m_iterator->second->ecp_td.ecp_reply == lib::TASK_TERMINATED )
        {
          //  sr_ecp_msg.message("w mp task terminated");
            return false;
        }
    }

    return true;
}

// ###############################################################
// Generator pusty. Faktyczna generacja trajektorii odbywa sie w ECP
// ###############################################################

empty::empty(task::base& _mp_task): generator (_mp_task)
{}

// ----------------------------------------------------------------------------------------------
// ---------------------------------    metoda	first_step -------------------------------------
// ----------------------------------------------------------------------------------------------

bool empty::first_step ()
{

    // Funkcja zwraca false gdy koniec generacji trajektorii
    // Funkcja zwraca true gdy generacja trajektorii bedzie kontynuowana
    // Inicjacja generatora trajektorii
    // printf("mp first step\n");
    // wait_for_ECP_pulse = true;
    for (map <lib::ROBOT_ENUM, common::robot*>::iterator robot_m_iterator = robot_m.begin();
            robot_m_iterator != robot_m.end(); robot_m_iterator++)
    {
        robot_m_iterator->second->ecp_td.mp_command = lib::NEXT_POSE;
        robot_m_iterator->second->ecp_td.instruction_type = lib::QUERY;
        robot_m_iterator->second->communicate = true;
    }

    return true;
}

// ----------------------------------------------------------------------------------------------
// -----------------------------------  metoda	next_step --------------------------------------
// ----------------------------------------------------------------------------------------------

bool empty::next_step ()
{
    // Funkcja zwraca false gdy koniec generacji trajektorii
    // Funkcja zwraca true gdy generacja trajektorii bedzie kontynuowana
    // Na podstawie ecp_reply dla poszczegolnych robotow nalezy okreslic czy
    // skonczono zadanie uzytkownika

    // obrazu danych wykorzystywanych przez generator

    // 	if (trigger) printf("Yh\n"); else printf("N\n");
    // printf("mp next step\n");
    // UWAGA: dzialamy na jednoelementowej liscie robotow

    for (map <lib::ROBOT_ENUM, common::robot*>::iterator robot_m_iterator = robot_m.begin();
            robot_m_iterator != robot_m.end(); robot_m_iterator++)
    {
        if ( robot_m_iterator->second->ecp_td.ecp_reply == lib::TASK_TERMINATED )
        {
            sr_ecp_msg.message("w mp task terminated");
            return false;
        }
    }

    return true;
}

delta::delta(task::base& _mp_task): generator (_mp_task)
{}

// ####################################################################################################
// Generator prostoliniowy o zadany przyrost polozenia/orientacji
// ####################################################################################################

tight_coop::tight_coop(task::base& _mp_task, lib::trajectory_description irp6ot_tr_des,
        lib::trajectory_description irp6p_tr_des): delta (_mp_task)
{
    irp6ot_td = irp6ot_tr_des;
    irp6p_td = irp6p_tr_des;
}

tight_coop::~tight_coop()
{ }

// ----------------------------------------------------------------------------------------------
// ---------------------------------    metoda	first_step -------------------------------------
// ----------------------------------------------------------------------------------------------

bool tight_coop::first_step ()
{
    // Generacja trajektorii prostoliniowej o zadany przyrost polozenia i orientacji
    // Funkcja zwraca false gdy koniec generacji trajektorii
    // Funkcja zwraca true gdy generacja trajektorii bedzie kontynuowana

    idle_step_counter = 2;

    for (map <lib::ROBOT_ENUM, common::robot*>::iterator robot_m_iterator = robot_m.begin();
            robot_m_iterator != robot_m.end(); robot_m_iterator++)
    {
        robot_m_iterator->second->ecp_td.mp_command = lib::NEXT_POSE;
        robot_m_iterator->second->ecp_td.instruction_type = lib::GET;
        robot_m_iterator->second->ecp_td.get_type = ARM_DV;
        robot_m_iterator->second->ecp_td.set_type = ARM_DV;
        robot_m_iterator->second->ecp_td.set_arm_type = lib::XYZ_EULER_ZYZ;
        robot_m_iterator->second->ecp_td.get_arm_type = lib::XYZ_EULER_ZYZ;
        robot_m_iterator->second->ecp_td.motion_type = lib::ABSOLUTE;
        robot_m_iterator->second->ecp_td.next_interpolation_type = lib::MIM;
        robot_m_iterator->second->ecp_td.motion_steps = irp6ot_td.internode_step_no;
        robot_m_iterator->second->ecp_td.value_in_step_no = irp6ot_td.value_in_step_no;
        robot_m_iterator->second->communicate = true;
    }

    return true;
}

// ----------------------------------------------------------------------------------------------
// -----------------------------------  metoda	next_step --------------------------------------
// ----------------------------------------------------------------------------------------------

bool tight_coop::next_step ()
{
    // Generacja trajektorii prostoliniowej o zadany przyrost polozenia i orientacji
    // Funkcja zwraca false gdy koniec generacji trajektorii
    // Funkcja zwraca true gdy generacja trajektorii bedzie kontynuowana
    // UWAGA: dzialamy na jednoelementowej liscie robotow
    int i; // licznik kolejnych wspolrzednych wektora [0..6]

    if ( idle_step_counter )
    { // Oczekiwanie na odczyt aktualnego polozenia koncowki
        idle_step_counter--;
        return true;
    }

    if (node_counter-1 == irp6ot_td.interpolation_node_no)
        return false;



    map <lib::ROBOT_ENUM, common::robot*>::iterator robot_m_iterator = robot_m.begin();

    // Przygotowanie kroku ruchu - do kolejnego wezla interpolacji
    robot_m_iterator->second->ecp_td.instruction_type = lib::SET;
    robot_m_iterator->second->ecp_td.get_type = NOTHING_DV;
    robot_m_iterator->second->ecp_td.get_arm_type = lib::INVALID_END_EFFECTOR;

    // Obliczenie zadanej pozycji posredniej w tym kroku ruchu
    // (okreslenie kolejnego wezla interpolacji)
    for (i = 0; i < 6; i++) // zakladamy, ze na liscie jest jeden robot
        robot_m_iterator->second->ecp_td.next_XYZ_ZYZ_arm_coordinates[i] =
            robot_m_iterator->second->ecp_td.current_XYZ_ZYZ_arm_coordinates[i]
            + node_counter * irp6ot_td.coordinate_delta[i] / irp6ot_td.interpolation_node_no;
    // printf("X_d= %lf  X_a= %lf\n",robot_list->E_ptr->ecp_td.next_XYZ_ZYZ_arm_coordinates[0],robot_list->E_ptr->ecp_td.current_XYZ_ZYZ_arm_coordinates[0]);
    // printf("Y_d= %lf  Y_a= %lf\n",robot_list->E_ptr->ecp_td.next_XYZ_ZYZ_arm_coordinates[1],robot_list->E_ptr->ecp_td.current_XYZ_ZYZ_arm_coordinates[1]);
    // printf("Z_d= %lf  Z_a= %lf\n",robot_list->E_ptr->ecp_td.next_XYZ_ZYZ_arm_coordinates[2],robot_list->E_ptr->ecp_td.current_XYZ_ZYZ_arm_coordinates[2]);

    robot_m_iterator->second->ecp_td.next_gripper_coordinate =
        robot_m_iterator->second->ecp_td.current_gripper_coordinate
        + node_counter * irp6ot_td.coordinate_delta[6] / irp6ot_td.interpolation_node_no;

    // by Y - ZAKOMENTOWANE ponizej - nie wiadomo jaka idea temu przyswiecala
    // ale dzialalo to zle z generatorami transparentnymi ECP
    /*
      if (node_counter == td.interpolation_node_no) {
        // Zakonczenie generacji trajektorii
        robot_list->E_ptr->ecp_td.mp_command = lib::END_MOTION; 
      }
    */

    if ((++robot_m_iterator) != robot_m.end())
    {
        // Przygotowanie kroku ruchu - do kolejnego wezla interpolacji
        robot_m_iterator->second->ecp_td.instruction_type = lib::SET;
        robot_m_iterator->second->ecp_td.get_type = NOTHING_DV;
        robot_m_iterator->second->ecp_td.get_arm_type = lib::INVALID_END_EFFECTOR;
        // Obliczenie zadanej pozycji posredniej w tym kroku ruchu
        // (okreslenie kolejnego wezla interpolacji)
        for (i = 0; i < 6; i++) // zakladamy, ze na liscie jest jeden robot
            robot_m_iterator->second->ecp_td.next_XYZ_ZYZ_arm_coordinates[i] =
                robot_m_iterator->second->ecp_td.current_XYZ_ZYZ_arm_coordinates[i]
                + node_counter * irp6p_td.coordinate_delta[i] / irp6p_td.interpolation_node_no;

        robot_m_iterator->second->ecp_td.next_gripper_coordinate =
            robot_m_iterator->second->ecp_td.current_gripper_coordinate
            + node_counter * irp6p_td.coordinate_delta[6] / irp6p_td.interpolation_node_no;

    }

    // skopiowac przygotowany rozkaz dla ECP do bufora wysylkowego

    return true;
}

} // namespace generator
} // namespace mp
} // namespace mrrocpp

