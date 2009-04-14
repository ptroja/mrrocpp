// -------------------------------------------------------------------------
//                             ecp_gen_test.cc
//             Effector Control Process (lib::ECP) - force & torque methods
// 			Test nowego EDP z wykorzystaniem sily
// 			By Slawek Bazant
//			Ostatnia modyfikacja: 05.01.2006r.
// -------------------------------------------------------------------------

#include <stdio.h>

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "lib/srlib.h"
#include "ecp/irp6_on_track/ecp_local.h"
#include "ecp/irp6_on_track/ecp_g_test.h"
#include "lib/mathtr.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot {
namespace generator {


bool y_simple::first_step ( )
{

    td.interpolation_node_no = 1;
    td.internode_step_no = step_no;
    td.value_in_step_no = td.internode_step_no - 2;


    the_robot->EDP_data.instruction_type = lib::GET;
    the_robot->EDP_data.get_type = ARM_DV; // arm - ORYGINAL
    the_robot->EDP_data.set_type = ARM_DV;
    the_robot->EDP_data.set_arm_type = POSE_FORCE_TORQUE_AT_FRAME;
    the_robot->EDP_data.get_arm_type = POSE_FORCE_TORQUE_AT_FRAME;
    the_robot->EDP_data.motion_type = lib::RELATIVE;
     the_robot->EDP_data.next_interpolation_type = lib::TCIM;
    the_robot->EDP_data.motion_steps = td.internode_step_no;
    the_robot->EDP_data.value_in_step_no = td.value_in_step_no;


    return true;
}
; // end: bool y_simple_generator::first_step (map <lib::SENSOR_ENUM, lib::sensor*>& sensor_m, robot& the_robot )
//--------------------------------------------------------------------------


// --------------------------------------------------------------------------
bool y_simple::next_step ( )
{
    //	struct timespec start[9];
    if (ecp_t->pulse_check())
    {
        ecp_t->mp_buffer_receive_and_send ();
        return false;
    }

    the_robot->EDP_data.instruction_type = lib::SET_GET;
    the_robot->EDP_data.set_type = ARM_DV;
    the_robot->EDP_data.get_type = ARM_DV;
    the_robot->EDP_data.get_arm_type = POSE_FORCE_TORQUE_AT_FRAME;
    for(int i =0; i<4; i++)
    {
        for(int j=0; j<3; j++)
        {
            if(i==j)
            {
                the_robot->EDP_data.ECPtoEDP_reference_frame[i][j] = 1;
                the_robot->EDP_data.ECPtoEDP_force_tool_frame[i][j] = 1;
            }
            else
            {
                the_robot->EDP_data.ECPtoEDP_reference_frame[i][j] = 0;
                the_robot->EDP_data.ECPtoEDP_force_tool_frame[i][j] = 0;
            }
        }
    }
    //	the_robot->EDP_data.ECPtoEDP_reference_frame[1][0] = 1;
    //	the_robot->EDP_data.ECPtoEDP_reference_frame[0][1] = 1;
    //	the_robot->EDP_data.ECPtoEDP_reference_frame[2][2] = -1;
    //	the_robot->EDP_data.ECPtoEDP_force_tool_frame[3][0] = 0.1;
    //	the_robot->EDP_data.ECPtoEDP_force_tool_frame[3][1] = 0.1;
    the_robot->EDP_data.ECPtoEDP_force_tool_frame[3][2] = 0.18;
    for(int i=0;i<6;i++)
    {
        the_robot->EDP_data.ECPtoEDP_pos_xyz_rot_xyz[i] = 0;
        the_robot->EDP_data.next_force_xyz_torque_xyz[i] = 0;
        the_robot->EDP_data.selection_vector[i] = FORCE_SV_AX;
    }
    //	the_robot->EDP_data.ECPtoEDP_force_xyz_torque_xyz[2] = -5;
    for (int i=2;i<5;i++)
        the_robot->EDP_data.selection_vector[i] = POSE_SV_AX;
    //	the_robot->EDP_data.ECPtoEDP_force_xyz_torque_xyz[5] = 8;
    the_robot->EDP_data.force_move_mode=0;
    if(the_robot->EDP_data.current_gripper_coordinate < 0.058)
        the_robot->EDP_data.next_gripper_coordinate = the_robot->EDP_data.current_gripper_coordinate;
    else
        the_robot->EDP_data.next_gripper_coordinate = the_robot->EDP_data.current_gripper_coordinate-0.0001;

    /*
    	frame_tab beggining_frame;
    	lib::copy_frame(beggining_frame,the_robot->EDP_data.current_arm_frame);
    	lib::Homog_matrix beg_frame = lib::Homog_matrix(beggining_frame);
    	cout << endl << "ecp: beginning_frame" << endl << endl << beg_frame;
    	frame_tab present_frame;
    	lib::copy_frame(present_frame,the_robot->EDP_data.current_arm_frame);
    	lib::Homog_matrix pres_frame = lib::Homog_matrix(present_frame);
    	cout << endl << "ecp: present_frame" << endl << endl << pres_frame;
    	frame_tab predicted_frame;
    	lib::copy_frame(predicted_frame,the_robot->EDP_data.current_arm_frame);
    	lib::Homog_matrix pred_frame = lib::Homog_matrix(predicted_frame);
    	cout << endl << "ecp: predicted_frame" << endl << endl<< pred_frame;
    	double force[6];
    	for (int i=0;i<6;i++) force[i] = the_robot->EDP_data.current_force_xyz_torque_xyz[i];
    	cout << "force" << endl << endl;
    	for(int i=0;i<6;i++) cout << force[i] << "  " ;
    	cout << endl;
    */
    //	if(++run_counter==1000) return false;
    return true;
}
; // end:  y_simple_generator::next_step (, robot& the_robot )

}
} // namespace irp6ot
} // namespace ecp
} // namespace mrrocpp

