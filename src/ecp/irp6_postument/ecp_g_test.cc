// -------------------------------------------------------------------------
//                            ecp.cc
//            Effector Control Process (ECP) - force methods
// Funkcje do tworzenia procesow ECP z wykorzystaniem sily
//
// Ostatnia modyfikacja: 2004r.
// -------------------------------------------------------------------------

#include <stdio.h>

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "lib/srlib.h"
#include "ecp/irp6_postument/ecp_local.h"
#include "ecp/irp6_postument/ecp_gen_test.h"

#include "lib/mathtr.h"

namespace mrrocpp {
namespace ecp {
namespace irp6p {
namespace generator {

y_simple::y_simple_generator(common::task::base& _ecp_task, int step)
        :
        base (_ecp_task, true)
{
    step_no = step;
};

bool y_simple::first_step ( )
{

    run_counter = 0;
    second_step = false;
    for (int j=0; j<3; j++)
        for (int i=0; i<4; i++)
            previous_frame[i][j]=0;

    for (int i=0; i<6; i++)
        delta[i]=0.0;


    td.interpolation_node_no = 1;
    td.internode_step_no = step_no;
    td.value_in_step_no = td.internode_step_no - 2;


    the_robot->EDP_data.instruction_type = SET_GET;
    the_robot->EDP_data.get_type = ARM_DV; // arm - ORYGINAL
    the_robot->EDP_data.set_type = RMODEL_DV;
    the_robot->EDP_data.set_arm_type = FRAME;
    the_robot->EDP_data.set_rmodel_type = TOOL_FRAME;
    for(int i=0; i<4; i++)
        for(int j=0; j<3; j++)
            if(i==j)
                the_robot->EDP_data.next_tool_frame[i][j]=1;
    //				else if (i==3 && j==1) the_robot->EDP_data.next_tool_frame[i][j]=-0.1;
            else if (i==3 && j==2)
                the_robot->EDP_data.next_tool_frame[i][j]= 0.2;
            else
                the_robot->EDP_data.next_tool_frame[i][j]=0;
    the_robot->EDP_data.get_arm_type = FRAME;
    the_robot->EDP_data.motion_type = ABSOLUTE;
     the_robot->EDP_data.next_interpolation_type = MIM;
    the_robot->EDP_data.motion_steps = td.internode_step_no;
    the_robot->EDP_data.value_in_step_no = td.value_in_step_no;

    return true;
}
; // end: bool y_nose_run_force_generator::first_step ( )
// --------------------------------------------------------------------------


// --------------------------------------------------------------------------
bool y_simple::next_step ( )
{
    struct timespec start[9];


    //	printf("bbb\n");
    if (ecp_t->pulse_check())
    {
        ecp_t->mp_buffer_receive_and_send ();
        return false;
    }

    the_robot->EDP_data.instruction_type = SET;
    the_robot->EDP_data.set_type = ARM_DV;
    the_robot->EDP_data.get_type = NOTHING_DV;
    the_robot->EDP_data.get_arm_type = INVALID_END_EFFECTOR;
    

    double axis_table[3][3] = {{1,0,0},{0,1,0},{0,0,1}};
    vector x_axis(axis_table[0]);
    vector y_axis(axis_table[1]);
    vector z_axis(axis_table[2]);
    lib::Homog_matrix curr_frame(the_robot->EDP_data.current_arm_frame);								// pobranie aktualnej ramki (tylko na poczatku ruchu)
    lib::Homog_matrix prev_frame(previous_frame);
    lib::Homog_matrix next_frame;
    lib::Homog_matrix temp = !curr_frame;
    lib::Homog_matrix temp2 = !prev_frame;																	// odwrocenie aktualnej ramki do mnozenia przez ramke obrotu
    temp2.move(0,0,0);																							// oczysczenie z wszelkiego ruchu
    temp.move(0,0,0);
    const double FORCE_TO_MOVE_RATIO = 0.00005;
    const double TORQUE_TO_ROTATE_RATIO = 0.001;
    //	frame_tab rotation = {{0,-1,0},{1,0,0},{0,0,1},{0,0,0}};
    //	lib::Homog_matrix sensor_rotation(rotation);
    double move_tab[3] = {0,0,0};
    bool force_mov[3] = {true,true,true};
    double mov[3] = { move_tab[0] + force_mov[0]*FORCE_TO_MOVE_RATIO*(sensor_m.begin())->second->image.sensor_union.force.rez[0] , 		// tablica zmiany pozycji wzgledem sil
                      move_tab[1] + force_mov[1]*FORCE_TO_MOVE_RATIO*(sensor_m.begin())->second->image.sensor_union.force.rez[1] ,
                      move_tab[2] + force_mov[2]*FORCE_TO_MOVE_RATIO*(sensor_m.begin())->second->image.sensor_union.force.rez[2]};
    vector move_vector(mov);
    lib::Homog_matrix temporary_frame;
    if(!second_step)
        temporary_frame = curr_frame;
    else
        temporary_frame = prev_frame;
    temporary_frame.move(0,0,0);
    move_vector =  temporary_frame * move_vector;
    move_vector.to_table(mov);
    double rot_tab[3] =
        {
            0,0,0/*.0003*/
        };
    bool force_rot[3] =
        {
            true,true,true
        };
    double rot[3] =
        {
            rot_tab[0] + force_rot[0]*TORQUE_TO_ROTATE_RATIO*(sensor_m.begin())->second->image.sensor_union.force.rez[3] , 		// tablica zmiany orientacji wzgledem momentow sill
            rot_tab[1] + force_rot[1]*TORQUE_TO_ROTATE_RATIO*(sensor_m.begin())->second->image.sensor_union.force.rez[4] ,
            rot_tab[2] + force_rot[2]*TORQUE_TO_ROTATE_RATIO*(sensor_m.begin())->second->image.sensor_union.force.rez[5]
        }
        ;
    lib::Homog_matrix move_frame(mov[0], mov[1], mov[2]);
    lib::Homog_matrix rot_frame(x_axis, y_axis, z_axis, rot);
    if(!second_step)
    {
        next_frame = move_frame * curr_frame * rot_frame;											// wyliczenie nowej ramki
        second_step = true;
    }
    else
        next_frame = move_frame * prev_frame * rot_frame;
    next_frame.to_table(previous_frame);

    lib::copy_frame(the_robot->EDP_data.next_arm_frame, previous_frame);
    // przepisanie nowej ramki do EDP
    the_robot->EDP_data.next_gripper_coordinate=the_robot->EDP_data.current_gripper_coordinate;

    // // 	if(++run_counter==1000) return false;
    return true;
}
; // end:  y_nose_run_force_generator::next_step ( )

}
} // namespace irp6p
} // namespace ecp
} // namespace mrrocpp
