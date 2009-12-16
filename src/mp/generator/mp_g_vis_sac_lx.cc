// -------------------------------------------------------------------------
//                              mp.cc
//
// MP Master Process - methods�for visual generators
//
// Last issue: 06.05.22
// -------------------------------------------------------------------------
// Funkcje do konstruowania procesow MP

#include <stdio.h>
#include <math.h>
#include <iostream>

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "lib/srlib.h"
#include "mp/mp.h"
#include "mp/generator/mp_g_vis_sac_lx.h"

namespace mrrocpp {
namespace mp {
namespace generator {

//globalne do wywalenia
/*
double alfa;
double beta;
double gammax;

double measure[6][5];
double measure_v[6][5];
double measure_a[6][5];
double measure_d[6][5];
double measure_d2[6][5];

double stearing[6][5];
double stearing_v[6][5];
double stearing_a[6][5];
double stearing_d[6][5];
double stearing_d2[6][5];

double pose[6][5];

double frame1[4][4];
double vec1[6];
double lasty;

double lastframe[4][4];

double measure_aux[6];
*/
//double measure_border_u[]={0.840, 0.150, 0.305, 0.606, 1.57, 3.12}; //Zmienic ogranicz Z
//double measure_border_d[]={0.570, -0.150, 0.185, -0.606, 1.267, 1.8}; //by na Z 0.078 gamma 2.237 //byla beta 1.32

double measure_border_u[]={1.090, 0.150, 0.305, 0.606, 1.57, 3.12}; //Zmienic ogranicz Z
double measure_border_d[]={0.82, -0.150, 0.155, -0.606, 1.267, 2.5}; // gamma 1.8


//double measure_border_u[]={1.000, 0.220, 0.338, 0.606, 1.57, 3.12}; //Zmienic ogranicz Z
//double measure_border_d[]={0.810, -0.250, 0.130, -0.606, 1.267, 1.8}; //by na Z 0.078 gamma 2.237 //byla beta 1.32



//double d_u_max[6]={0.010, 0.0250, 0.010, 0.02, 0.010, 0.04}; //td.internode_step_no = 80;
//double d2_u_max[6]={0.002, 0.002, 0.002, 0.02, 0.01, 0.01}; //tylko ustalone dla Y

//0.01

double d_u_max[6]={0.3, 0.3, 0.010, 0.3, 0.3, 0.3}; //td.internode_step_no = 80; //jeszcze nie w [m]
double d2_u_max[6]={0.1, 0.1, 0.002, 0.05, 0.05, 0.05}; //tylko ustalone dla Y

//double d_u_max[6]={0.10, 0.10, 0.10, 0.05, 0.05, 0.05}; //td.internode_step_no = 80;
//double d2_u_max[6]={0.01, 0.01, 0.01, 0.01, 0.01, 0.01}; //tylko ustalone dla Y


double x2g=-0.07; //x nibytoola

int vis_phase = 0;
int steps2switch=0;

/*
int valid_measure;





double aux=0;
double aux2=0;
*/

#define PRINTB 0
#define PRINTA 0


vis_sac_lx::vis_sac_lx(task::task& _mp_task, int step): generator (_mp_task), irp6ot_con(1), irp6p_con(1)
{
    step_no = step;
};



// ----------------------------------------------------------------------------------------------
// ---------------------------------    metoda	first_step -------------------------------------
// ----------------------------------------------------------------------------------------------

bool vis_sac_lx::first_step ()
{
    // Generacja trajektorii prostoliniowej o zadany przyrost polozenia i oreintacji
    // Funkcja zwraca false gdy koniec generacji trajektorii
    // Funkcja zwraca true gdy generacja trajektorii bedzie kontynuowana
    printf("EYE GEN XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX\n\n\n\n\n\n\n\n\n\n\n");


    //	for (int i=0; i<6; i++)
    //  		for (int j=0; j<5; j++)
    //			measure[i][j]=0;

    irp6ot = robot_m[lib::ROBOT_IRP6_ON_TRACK];

    vsp_vis_sac = sensor_m[lib::SENSOR_CAMERA_SA];


    idle_step_counter = 1;
    vsp_vis_sac->base_period=0; //1
    vsp_vis_sac->current_period=0; //MAC7
    //vsp_force_irp6p->base_period=1;
    //td.interpolation_node_no = 1; //potrzebne? MAC7
    td.internode_step_no = 80; //step_no; 40
    td.value_in_step_no = td.internode_step_no - 10; //2

    //TOOL
    irp6ot->mp_command.instruction.rmodel.tool_frame_def.tool_frame[0][0]=1;
    irp6ot->mp_command.instruction.rmodel.tool_frame_def.tool_frame[1][0]=0;
    irp6ot->mp_command.instruction.rmodel.tool_frame_def.tool_frame[2][0]=0;
    irp6ot->mp_command.instruction.rmodel.tool_frame_def.tool_frame[0][3]=0;

    irp6ot->mp_command.instruction.rmodel.tool_frame_def.tool_frame[0][1]=0;
    irp6ot->mp_command.instruction.rmodel.tool_frame_def.tool_frame[1][1]=1;
    irp6ot->mp_command.instruction.rmodel.tool_frame_def.tool_frame[2][1]=0;
    irp6ot->mp_command.instruction.rmodel.tool_frame_def.tool_frame[1][3]=0;

    irp6ot->mp_command.instruction.rmodel.tool_frame_def.tool_frame[0][2]=0;
    irp6ot->mp_command.instruction.rmodel.tool_frame_def.tool_frame[1][2]=0;
    irp6ot->mp_command.instruction.rmodel.tool_frame_def.tool_frame[2][2]=1;
    irp6ot->mp_command.instruction.rmodel.tool_frame_def.tool_frame[2][3]=0.25; //0.25;


    irp6ot->mp_command.command = lib::NEXT_POSE;
    irp6ot->mp_command.instruction.instruction_type = lib::SET_GET; //GET;
    irp6ot->mp_command.instruction.get_type = RMODEL_DV; //ARM_DV;
    irp6ot->mp_command.instruction.set_type = RMODEL_DV; //ARM_DV;
    //irp6ot->mp_command.instruction.set_arm_type = lib::FRAME; //XYZ_EULER_ZYZ; //POSE_FORCE_TORQUE_AT_FRAME;
    //irp6ot->mp_command.instruction.set_arm_type = lib::XYZ_ANGLE_AXIS;
    /*
    irp6ot->mp_command.instruction.set_arm_type = lib::XYZ_ANGLE_AXIS;
*/
    irp6ot->mp_command.instruction.get_arm_type = lib::FRAME;
    //FRAME; //XYZ_EULER_ZYZ; //POSE_FORCE_TORQUE_AT_FRAME;
    irp6ot->mp_command.instruction.set_rmodel_type = lib::TOOL_FRAME;
    irp6ot->mp_command.instruction.get_rmodel_type = lib::TOOL_FRAME;
    //irp6ot->mp_command.instruction.motion_type = lib::ABSOLUTE; //RELATIVE;
    irp6ot->mp_command.instruction.motion_type = lib::ABSOLUTE; //PF_FIXED_FRAME_WITH_DESIRED_FORCE_OR_SPEED;
    irp6ot->mp_command.instruction.interpolation_type = lib::TCIM;

    irp6ot->mp_command.instruction.motion_steps = td.internode_step_no;
    irp6ot->mp_command.instruction.value_in_step_no = td.value_in_step_no;

    for(int i=0;i<6;i++)
    {
        irp6ot->mp_command.instruction.arm.pf_def.arm_coordinates[i] = 0;
        //		irp6ot->ecp_td.MPtoECP_force_xyz_torque_xyz[i] = 0;
        //	irp6ot->ecp_td.MPselection_vector[i] = FORCE_SV_AX;
        //		 irp6ot->ecp_td.MPselection_vector[i] = POSE_SV_AX;
    }



    for (int i=0;i<3;i++)
    {
        irp6ot->mp_command.instruction.arm.pf_def.inertia[i] = FORCE_INERTIA;
        irp6ot->mp_command.instruction.arm.pf_def.inertia[i+3] = TORQUE_INERTIA;
        irp6ot->mp_command.instruction.arm.pf_def.reciprocal_damping[i] = FORCE_RECIPROCAL_DAMPING;
        irp6ot->mp_command.instruction.arm.pf_def.reciprocal_damping[i+3] = TORQUE_RECIPROCAL_DAMPING;
        irp6ot->mp_command.instruction.arm.pf_def.behaviour[i] = lib::GUARDED_MOTION;
        irp6ot->mp_command.instruction.arm.pf_def.behaviour[i+3] = lib::GUARDED_MOTION;
    }


    return true;
}
; // end: vis_sac_lx::first_step()

// ----------------------------------------------------------------------------------------------
// -----------------------------------  metoda	next_step -----------------------------------
// ----------------------------------------------------------------------------------------------

bool vis_sac_lx::next_step ()
{
    // Generacja trajektorii prostoliniowej o zadany przyrost polozenia i orientacji
    // Funkcja zwraca false gdy koniec generacji trajektorii
    // Funkcja zwraca true gdy generacja trajektorii bedzie kontynuowana
    // UWAGA: dzialamy na jednoelementowej liscie robotow



    if (check_and_null_trigger())
        return false;

    if(node_counter==1 || node_counter==2)
    {
        irp6ot->mp_command.instruction.instruction_type = lib::GET;
        irp6ot->mp_command.instruction.get_type = ARM_DV | RMODEL_DV;

        O_Tx_E.set_frame_tab(irp6ot->ecp_reply_package.reply_package.arm.pf_def.arm_frame); // zarem

        //O_Tx_E.get_xyz_angle_axis(O_r_E[0]);

        std::cout << "MMM " << node_counter << std::endl;
        std::cout << "YYY " << 	O_Tx_E  << std::endl;

        // UWAGA: dzialamy na jednoelementowej liscie robotow
        if ( irp6ot->ecp_reply_package.reply == lib::TASK_TERMINATED )
        {
            sr_ecp_msg.message("w mp task terminated");
            return false;
        }
        else
            return true;

        return true;
    }

    if(node_counter!=1 && node_counter!=2)
    {
        irp6ot->mp_command.instruction.set_type = ARM_DV;
        irp6ot->mp_command.instruction.instruction_type = lib::SET_GET;
        //irp6ot->mp_command.instruction.get_type = NOTHING_DV;
        //irp6ot->mp_command.instruction.get_arm_type = lib::INVALID_END_EFFECTOR;
        //irp6ot->mp_command.instruction.get_arm_type = lib::INVALID_END_EFFECTOR;
    }

    if(node_counter==1 || node_counter==2 || node_counter==3 || node_counter==4)
    {
        O_Tx_E.set_frame_tab(irp6ot->ecp_reply_package.reply_package.arm.pf_def.arm_frame); // zarem
        std::cout << "YYY " << 	O_Tx_E  << std::endl;


        //O_Tx_E.set_frame_tab(irp6ot->ecp_reply_package.reply_package.arm.pf_def.arm_frame); // zarem
        O_Tx_E.get_xyz_angle_axis(O_r_E[0]);

        std::cout << "MMM " << node_counter << std::endl;
        std::cout << "YYY " << 	O_Tx_E  << std::endl;


        //	std::cout << O_Tx_E << std::endl;

        for (int i=0; i<6; i++)
        {

            O_r_Ep[0][i]=O_r_E[0][i];
            O_r_Ep[1][i]=O_r_E[0][i];
            O_r_Ep[2][i]=O_r_E[0][i];

            O_r_Ep_d[i][1]=0;
            O_r_Ep_d[i][2]=0;
        }
        for (int i=0; i<6; i++)
            vsp_vis_sac->image.sensor_union.vis_sac.frame_E_r_G__f[i]=O_r_E[0][i]; //nie wiem czy potrzebne bo chyba  robot sie nie rusza

        std::cout << node_counter <<"------------------------------------------------------------------" << std::endl;
    }



    //0_T_E=
    //-0.001652 0.000348 0.999999 0.950214
    //0.717719 -0.696331 0.001428 0.000358
    //0.696331 0.717721 0.000901 0.265080



    // x+=10cm
    /*
    my_goal_frame[0][0]=-0.001276; my_goal_frame[1][0]=-0.000117; my_goal_frame[2][0]=0.999999; my_goal_frame[3][0]=1.000204;
    my_goal_frame[0][1]=0.717735; my_goal_frame[1][1]=-0.696316; my_goal_frame[2][1]=0.000834; my_goal_frame[3][1]=0.000053;
    my_goal_frame[0][2]=0.696315; my_goal_frame[1][2]=0.717735; my_goal_frame[2][2]=0.000973; my_goal_frame[3][2]=0.264818;
    */
    //	O_Tx_G.set_frame_tab(my_goal_frame);

    /*
    std::cout << "C_T_G" << std::endl;
     	for(int i=0; i<4; i++)
    	{
    		for(int j=0; j<4; j++)
    		{
    			std::cout << vsp_vis_sac->image.sensor_union.vis_sac.frame_E_T_G[4*i+j];
    		}
    		std::cout << std::endl;
    	}
    */

    //std::cout << "C_T_G" << std::endl;
    //	for(int j=0; j<6; j++)
    //		{
    //			std::cout << vsp_vis_sac->image.sensor_union.vis_sac.frame_E_r_G__f[j] << ",";
    //		}
    //	std::cout << std::endl;





    C_Tx_G.set_xyz_rpy(vsp_vis_sac->image.sensor_union.vis_sac.frame_E_r_G__f[0],vsp_vis_sac->image.sensor_union.vis_sac.frame_E_r_G__f[1],
                       -vsp_vis_sac->image.sensor_union.vis_sac.frame_E_r_G__f[2],
                       vsp_vis_sac->image.sensor_union.vis_sac.frame_E_r_G__f[5]
                       ,0,0);
    //vsp_vis_sac->image.sensor_union.vis_sac.frame_E_r_G__f[5]);

    //std::cout << "MP C_T_G" << std::endl;
    //std::cout << C_Tx_G;

    //O_Tx_G.set_xyz_euler_zyz(0.950, 0.000, 0.265, 0.002, 1.481, 2.341);

    //	O_Tx_G.set_xyz_euler_zyz(0.950+0.058+vsp_vis_sac->image.sensor_union.vis_sac.frame_E_r_G__f[0],
    //	0.000+vsp_vis_sac->image.sensor_union.vis_sac.frame_E_r_G__f[1]-0.06,
    // 	0.265+vsp_vis_sac->image.sensor_union.vis_sac.frame_E_r_G__f[2]+0.900,
    //	0.002, 1.481+0.03, 2.341);

    //std::cout << "MP O_T_G pierwsze" << std::endl;
    //std::cout << O_Tx_G;

    //podjazd gdy sie nie ruszamy
    if(fabs(O_r_G[1][0]-O_r_G[0][0])<=0.01 && fabs(O_r_G[1][1]-O_r_G[0][1])<=0.01) //0.007
    {
        steps2switch++;
    }
    else
    {
        x2g=-0.07;
        steps2switch=0;
    }

    if(steps2switch>15) // to approach 15
        vis_phase=1;


    if(vis_phase)
    {
        x2g+=0.002; //0.001;
    }

    if(x2g>=0.0)
    {
        x2g=0.0; //0.02
        //return true; //to avoid graspin'
        //return false;	//to grasp
    }


    //odjazd gdy sie ruszymy
    //	if(fabs(measure[0][1]-measure[0][0])>=0.050 || fabs(measure[1][1]-measure[1][0])>=0.050 || fabs(measure[0][1]-measure[0][0])>=0.050)
    //	{
    //		x2g=-0.1;
    //		vis_phase=0;
    //		steps2switch=0;
    //	}

    x2g=-0.07;

    G_Tx_S.set_xyz_rpy(x2g, 0, 0, 0, 0, 0);

    G_Tx_G2.set_xyz_euler_zyz( 0,0,0, 0.002, 1.481+0.03, 2.341);

    O_Tx_C.set_xyz_rpy( 0.950+0.058, //-0.09,
                        0.000-0.06,
                        0.265+0.900+0.05,
                        0,0,0);

    //std::cout << "MP O_T_C" << std::endl;
    //std::cout << O_Tx_C;
    std::cout << "MP C_T_G" << std::endl;
    std::cout << C_Tx_G;


    O_Tx_G=O_Tx_C*C_Tx_G;

    //std::cout << "MP O_T_G2" << std::endl;
    //std::cout << O_Tx_G;

    std::cout << "MP O_T_G1" << std::endl;
    std::cout << O_Tx_G;


    //skrot myslowy
    O_Tx_G=O_Tx_G*G_Tx_S;

    std::cout << "MP O_T_G2" << std::endl;
    std::cout << O_Tx_G;
    O_Tx_G=O_Tx_G*G_Tx_G2;
    std::cout << "MP O_T_G3" << std::endl;
    std::cout << O_Tx_G;


    //std::cout << "MP O_T_G" << std::endl;
    //std::cout << O_Tx_G;

    O_Tx_G.get_xyz_angle_axis(O_r_G[0]);

    O_Tx_E.set_frame_tab(irp6ot->ecp_reply_package.reply_package.arm.pf_def.arm_frame);
    O_Tx_E.get_xyz_angle_axis(O_r_E[0]);


    //	std::cout << "O_r_Ep[0][i]  O_r_Ep[1][i] O_eps_EG[0][i]   O_r_G[0][i] O_r_E[0][i]" << std::endl;
    std::cout << "poczatek ";
    std::cout << O_r_E[0][1] << " " <<  O_r_E[1][1] << std::endl;


    for (int i=0; i<2; i++)
    {
        O_eps_EG[0][i]=O_r_G[0][i]-O_r_E[0][i];
        O_r_Ep[0][i]=O_r_E[0][i]+0.5*O_eps_EG[0][i]; //0.01
        std::cout << "O_r_Ep[0][i]  O_r_E[0][i] O_eps_EG[0][i]  O_r_G[0][i] O_r_E[0][i]" << std::endl;

        std::cout << O_r_Ep[0][i]  << " " <<  O_r_E[0][i] << " " << O_eps_EG[0][i] << " " <<  O_r_G[0][i] << " " << O_r_E[0][i] << std::endl;
    }

    for (int i=3; i<6; i++)
    {
        O_eps_EG[0][i]=O_r_G[0][i]-O_r_E[0][i];
        O_r_Ep[0][i]=O_r_E[0][i]+0.5*O_eps_EG[0][i]; //0.01
        //		std::cout << O_r_Ep[0][i]  << " " <<  O_r_E[0][i] << " " << O_eps_EG[0][i] << " " <<  O_r_G[0][i] << " " << O_r_E[0][i] << std::endl;
    }
    //O_r_Ep[0][0]=O_r_E[0][0]+0.005*O_eps_EG[0][0];
    /*
    O_r_Ep[0][2]=O_r_E[0][2]+0.01*O_eps_EG[0][2];
    O_r_Ep[0][3]=O_r_E[0][3]+0.1*O_eps_EG[0][3];
    */
    //O_r_Ep[0][0]=O_r_E[0][0];
    //O_r_Ep[0][1]=O_r_E[0][1];
    O_r_Ep[0][2]=O_r_E[0][2];

    if(node_counter==1 || node_counter==2 || node_counter==3 || node_counter==4)
    {
        for (int i=0; i<6; i++)
        {
            O_r_Ep[0][i]=O_r_E[0][i];
        }
    }


    //O_r_Ep[0][4]=O_r_E[0][4]+0.002*O_eps_EG[0][4];
    //O_r_Ep[0][5]=O_r_E[0][5]+0.002*O_eps_EG[0][5];
    //


    std::cout << "przed kranc ";
    for (int i=0; i<6; i++)
    {
        std::cout << O_r_Ep[0][i] << " ";
    }

    std::cout << std::endl;

    // roznica w kroku -> docelowo predkosc
    for(int i=0; i<6; i++)
        O_r_Ep_d[0][i]=O_r_Ep[0][i]-O_r_E[0][i];

    // roznica w 2 krokach -> docelowo przyspieszenie
    for(int i=0; i<6; i++)
        O_r_Ep_d2[0][i]=O_r_Ep_d[0][i]-O_r_Ep_d[1][i];



    //ograniczenie przyspieszenia (opoznienie zalatwia regulator proporcjonalny)
    for(int i=0; i<6; i++)
    {
        if((fabs(O_r_Ep_d2[0][i])>=d2_u_max[i]) && (fabs(O_r_Ep_d[0][i])>=fabs(O_r_Ep_d[1][i])) )
        {
            if(O_r_Ep_d[0][i]>=0)
            {
                O_r_Ep_d[0][i]=O_r_Ep_d[1][i]+d2_u_max[i];
                O_r_Ep[0][i]=O_r_E[0][i]+O_r_Ep_d[0][i];
            }
            else
                if(O_r_Ep_d[0][i]<0)
                {
                    O_r_Ep_d[0][i]=O_r_Ep_d[1][i]-d2_u_max[i];
                    O_r_Ep[0][i]=O_r_E[0][i]+O_r_Ep_d[0][i];
                }
        }
    }

    std::cout << "po aaa ";
    for (int i=0; i<6; i++)
    {
        std::cout << O_r_Ep[0][i] << " ";
    }

    std::cout << std::endl;

    std::cout << "po aaa2 ";
    std::cout << O_r_Ep[0][1] << " " <<  O_r_Ep[1][1] << std::endl;


    //ograniczenie predkosci
    for(int i=0; i<6; i++)
    {
        if(O_r_Ep_d[0][i]>=d_u_max[i])
        {
            O_r_Ep[0][i]=O_r_E[0][i]+d_u_max[i];

        }

        if(O_r_Ep_d[0][i]<=-1*d_u_max[i])
        {
            O_r_Ep[0][i]=O_r_E[0][i]-d_u_max[i];

        }
    }


    std::cout << "po vvv ";
    for (int i=0; i<6; i++)
    {
        std::cout << O_r_Ep[0][i] << " ";
    }
    std::cout << std::endl;

    //ograniczenie przyspieszenia
    for(int i=0; i<6; i++)
    {
        O_r_Ep_d2[i][0]=O_r_Ep_d[i][0]-O_r_Ep_d[i][1];
    }

    //krancowki na sterowaniu - nie wiemy jakie krancowki dla angle axis
    for(int i=0; i<3; i++)
    {
        if(O_r_Ep[i][0]<measure_border_d[i])
            O_r_Ep[i][0]=measure_border_d[i];

        if(O_r_Ep[i][0]>measure_border_u[i])
            O_r_Ep[i][0]=measure_border_u[i];
    }

    //std::cout << std::endl;
    //std::cout << "po krans " << std::endl;
    //	for (int i=0; i<6; i++)
    //	{

    //		std::cout << O_r_Ep[0][i]  <<  " ";
    //	}

    //O_r_Ep[0][1]=0.05;

    O_Tx_Ep.set_xyz_angle_axis(O_r_Ep[0]);
    /*
    O_Tx_Ep.get_xyz_euler_zyz(O_reul_Ep[0]);

    //krancowki na sterowaniu - dla katow w eulerze
    for(int i=3; i<6; i++)
{
     	if(O_reul_Ep[i][0]<measure_border_d[i])
     		O_reul_Ep[i][0]=measure_border_d[i];

     	if(O_reul_Ep[i][0]>measure_border_u[i])
     		O_reul_Ep[i][0]=measure_border_u[i];
}

    O_Tx_Ep.set_xyz_euler_zyz(O_reul_Ep[0][0],O_reul_Ep[1][0],O_reul_Ep[2][0],O_reul_Ep[3][0],O_reul_Ep[4][0],O_reul_Ep[5][0]);
    O_Tx_Ep.get_xyz_angle_axis(O_r_Ep[0]);
    */


    std::cout << "po kranc ";
    for (int i=0; i<6; i++)
    {
        std::cout << O_r_Ep[0][i] <<" ";
    }


    E_Tx_Ep = !O_Tx_E * O_Tx_Ep;


    E_Tx_Ep.get_xyz_angle_axis(E_r_Ep[0]);

    //O_Tx_Ep.get_frame_tab(irp6ot->mp_command.instruction.arm.pf_def.arm_frame); //jesli sie chcemy ruszyc

    //	std::cout << "O_Tx_Ep" <<std::endl;
    //	std::cout << O_Tx_Ep << std::endl;

    //	std::cout << "E_Tx_Ep" <<std::endl;
    //	std::cout << E_Tx_Ep << std::endl;

    //	std::cout << "ErEp" << std::endl;



    //	for(int i=0; i<6; i++)
    //		std::cout << E_r_Ep[0][i] << " ";

    //	std::cout << std::endl;

    // ------------przepisanie wartosci-----
    for (int i=0; i<6; i++)
    {
        O_r_Ep[2][i]=O_r_Ep[1][i];
        O_r_Ep[1][i]=O_r_Ep[0][i];
        O_r_Ep_d[i][2]=O_r_Ep_d[i][1];
        O_r_Ep_d[i][1]=O_r_Ep_d[i][0];

        O_r_G[1][i]=O_r_G[0][i];
    }



    //	O_Tx_Ep.get_frame_tab( irp6ot->mp_command.instruction.arm.pf_def.arm_frame); //jesli ABSOLUTNIE zadajemy O_Tx_E

    if(node_counter==4)
    {
        vsp_vis_sac->base_period=1;
        vsp_vis_sac->current_period=1; //1
    }


    //pose regulator
    /*
    for (int i=0; i<6; i++)
{
    	irp6ot->mp_command.instruction.arm.pf_def.arm_coordinates[i]=E_r_Ep[0][i];
}
     */


    //force regulator
#if 0
    //sterujemy predkoscia
    for(int i=0;i<3; i++) {

    	irp6ot->ecp_td.MPtoECP_position_velocity[i] = 0; //1.5*E_r_Ep[0][i];//*2*td.internode_step_no;
}

    for(int i=3;i<5;i++) {

    	irp6ot->ecp_td.MPtoECP_position_velocity[i] = 0; //0.9*E_r_Ep[0][i];//*2*td.internode_step_no;
}

    for(int i=0;i<3; i++) {

    	irp6ot->ecp_td.MPtoECP_position_velocity[i] = E_r_Ep[0][i]*
    	(double) (1/ ( ((double)STEP)*((double)step_no)*1) ); //1.5*E_r_Ep[0][i];//*2*td.internode_step_no;
}

    for(int i=3;i<5;i++) {

    	irp6ot->ecp_td.MPtoECP_position_velocity[i] = E_r_Ep[0][i]*
    	(double) (1/ ( ((double)STEP)*((double)step_no)*1) );//*2*td.internode_step_no;
}
#endif

    //sterujemy polozeniem


    for(int i=0;i<6;i++)
    {
        irp6ot->mp_command.instruction.arm.pf_def.arm_coordinates[i] = O_r_Ep[0][i];
    }
    /*
    for(int i=0;i<6;i++) {
    		 irp6ot->ecp_td.MPtoECP_force_xyz_torque_xyz[i] = -E_r_Ep[0][i]*5000;
    		  irp6ot->ecp_td.MPtoECP_force_xyz_torque_xyz[i] = -E_r_Ep[0][i]*100;


}

    */
    //	if(irp6ot_con) irp6p->ecp_td.MPselection_vector[i] = FORCE_SV_AX;
    //	else
    // irp6ot->ecp_td.MPselection_vector[i] = POSE_SV_AX;


    //	}


    irp6ot->mp_command.instruction.arm.pf_def.gripper_coordinate=irp6ot->ecp_reply_package.reply_package.arm.pf_def.gripper_coordinate;

    /*******************************************************/


    // UWAGA: dzialamy na jednoelementowej liscie robotow
    if ( irp6ot->ecp_reply_package.reply == lib::TASK_TERMINATED )
    {
        sr_ecp_msg.message("w mp task terminated");
        return false;
    }
    else
        return true;
}
; // end: bool tight_coop_generator::next_step ()


} // namespace generator
} // namespace mp
} // namespace mrrocpp
