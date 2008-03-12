// -------------------------------------------------------------------------
//                              mp.cc
//
// MP Master Process - methods for visual generators
//
// Last issue: 06.05.22
// -------------------------------------------------------------------------
// Funkcje do konstruowania procesow MP

#include <stdio.h>
#include <math.h>
#include <iostream>

#include "common/typedefs.h"
#include "common/impconst.h"
#include "common/com_buf.h"

#include "lib/srlib.h"

#include "ecp/irp6_on_track/ecp_local.h"
#include "ecp/irp6_on_track/ecp_g_vis_sac_lx.h"

/*
double measure_border_u[]={1.090, 0.150, 0.305, 0.606, 1.57, 3.12}; //Zmienic ogranicz Z
double measure_border_d[]={0.82, -0.150, 0.155, -0.606, 1.267, 2.5}; // gamma 1.8
 
 
double d_u_max[6]={0.3, 0.3, 0.010, 0.3, 0.3, 0.3}; //td.internode_step_no = 80; //jeszcze nie w [m]
double d2_u_max[6]={0.1, 0.1, 0.01, 0.05, 0.05, 0.05}; //tylko ustalone dla Y
 
 
double x2g=-0.07; //x nibytoola
 
int vis_phase = 0;
int steps2switch=0;
*/


#define PRINTB 0
#define PRINTA 0


ecp_vis_sac_lx_generator::ecp_vis_sac_lx_generator (ecp_task& _ecp_task, int step): ecp_generator (_ecp_task)
{
    step_no = step;


    /*
    	measure_border_u[0]=1.090;
    	measure_border_u[1]=0.150;
    	measure_border_u[2]=0.305;
    	measure_border_u[3]=0.606;
    	measure_border_u[4]=1.57;
    	measure_border_u[5]=3.12;
     
    	measure_border_d[0]=0.82;
    	measure_border_d[1]=-0.150;
    	measure_border_d[2]=0.155;
    	measure_border_d[3]=-0.606;
    	measure_border_d[4]=1.267;
    	measure_border_d[5]=2.5;
     
    	d_u_max[0]=0.3;
    	d_u_max[1]=0.3;
    	d_u_max[2]=0.1;
    	d_u_max[3]=0.3;
    	d_u_max[4]=0.3;
    	d_u_max[5]=0.3;
     
    	d2_u_max[0]=0.1;
    	d2_u_max[1]=0.1;
    	d2_u_max[2]=0.01;
    	d2_u_max[3]=0.05;
    	d2_u_max[4]=0.05;
    	d2_u_max[5]=0.05;
     
    	gain[0]=0.5;
    	gain[1]=0.5;
    	gain[2]=0.2;
    	gain[3]=0.5;
    	gain[4]=0.5;
    	gain[5]=0.5;
    */

    measure_border_u[0]=ecp_t.config.return_double_value("measure_border_u0");
    measure_border_u[1]=ecp_t.config.return_double_value("measure_border_u1");
    measure_border_u[2]=ecp_t.config.return_double_value("measure_border_u2");
    measure_border_u[3]=ecp_t.config.return_double_value("measure_border_u3");
    measure_border_u[4]=ecp_t.config.return_double_value("measure_border_u4");
    measure_border_u[5]=ecp_t.config.return_double_value("measure_border_u5");

    measure_border_d[0]=ecp_t.config.return_double_value("measure_border_d0");
    measure_border_d[1]=ecp_t.config.return_double_value("measure_border_d1");
    measure_border_d[2]=ecp_t.config.return_double_value("measure_border_d2");
    measure_border_d[3]=ecp_t.config.return_double_value("measure_border_d3");
    measure_border_d[4]=ecp_t.config.return_double_value("measure_border_d4");
    measure_border_d[5]=ecp_t.config.return_double_value("measure_border_d5");

    d_u_max[0]=ecp_t.config.return_double_value("d_u_max0");
    d_u_max[1]=ecp_t.config.return_double_value("d_u_max1");
    d_u_max[2]=ecp_t.config.return_double_value("d_u_max2");
    d_u_max[3]=ecp_t.config.return_double_value("d_u_max3");
    d_u_max[4]=ecp_t.config.return_double_value("d_u_max4");
    d_u_max[5]=ecp_t.config.return_double_value("d_u_max5");

    d2_u_max[0]=ecp_t.config.return_double_value("d2_u_max0");
    d2_u_max[1]=ecp_t.config.return_double_value("d2_u_max1");
    d2_u_max[2]=ecp_t.config.return_double_value("d2_u_max2");
    d2_u_max[3]=ecp_t.config.return_double_value("d2_u_max3");
    d2_u_max[4]=ecp_t.config.return_double_value("d2_u_max4");
    d2_u_max[5]=ecp_t.config.return_double_value("d2_u_max5");

    gain[0]=ecp_t.config.return_double_value("gain0");
    gain[1]=ecp_t.config.return_double_value("gain1");
    gain[2]=ecp_t.config.return_double_value("gain2");
    gain[3]=ecp_t.config.return_double_value("gain3");
    gain[4]=ecp_t.config.return_double_value("gain4");
    gain[5]=ecp_t.config.return_double_value("gain5");

    force_inertia_=ecp_t.config.return_double_value("force_inertia_");
    torque_inertia_=ecp_t.config.return_double_value("torque_inertia_");
    force_reciprocal_damping_=ecp_t.config.return_double_value("force_reciprocal_damping_");
    torque_reciprocal_damping_=ecp_t.config.return_double_value("torque_reciprocal_damping_");

    x2g=ecp_t.config.return_double_value("x2g"); //x nibytoola

    vis_phase = 0;
    steps2switch=0;

};


// ----------------------------------------------------------------------------------------------
// ---------------------------------    metoda	first_step -------------------------------------
// ----------------------------------------------------------------------------------------------

bool ecp_vis_sac_lx_generator::first_step ()
{
    // Generacja trajektorii prostoliniowej o zadany przyrost polozenia i oreintacji
    // Funkcja zwraca false gdy koniec generacji trajektorii
    // Funkcja zwraca true gdy generacja trajektorii bedzie kontynuowana
    printf("EYE GEN XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX\n\n\n\n\n\n\n\n\n\n\n");


    //	for (int i=0; i<6; i++)
    //  		for (int j=0; j<5; j++)
    //			measure[i][j]=0;

    //	the_robot = robot_m[ROBOT_IRP6_ON_TRACK];

    vsp_vis_sac = sensor_m[SENSOR_CAMERA_SA];


    idle_step_counter = 1;
    vsp_vis_sac->base_period=0; //1
    vsp_vis_sac->current_period=0; //MAC7
    //vsp_force_irp6p->base_period=1;
    //td.interpolation_node_no = 1; //potrzebne? MAC7
    td.internode_step_no = 80; //step_no; 40
    td.value_in_step_no = td.internode_step_no - 10; //2

    //TOOL

    the_robot->EDP_data.next_tool_frame[0][0]=1;
    the_robot->EDP_data.next_tool_frame[1][0]=0;
    the_robot->EDP_data.next_tool_frame[2][0]=0;
    the_robot->EDP_data.next_tool_frame[0][3]=0;

    the_robot->EDP_data.next_tool_frame[0][1]=0;
    the_robot->EDP_data.next_tool_frame[1][1]=1;
    the_robot->EDP_data.next_tool_frame[2][1]=0;
    the_robot->EDP_data.next_tool_frame[1][3]=0;

    the_robot->EDP_data.next_tool_frame[0][2]=0;
    the_robot->EDP_data.next_tool_frame[1][2]=0;
    the_robot->EDP_data.next_tool_frame[2][2]=1;
    the_robot->EDP_data.next_tool_frame[2][3]=0.25; //0.25;


    //	the_robot->EDP_data.mp_command = NEXT_POSE;
    the_robot->EDP_data.instruction_type = SET_GET; //GET;
    the_robot->EDP_data.get_type = ARM_DV;
    the_robot->EDP_data.set_type = RMODEL_DV; //ARM_DV;
    //the_robot->EDP_data.set_arm_type = FRAME; //XYZ_EULER_ZYZ; //POSE_FORCE_TORQUE_AT_FRAME;
    //the_robot->EDP_data.set_arm_type = XYZ_ANGLE_AXIS;
    the_robot->EDP_data.set_arm_type = XYZ_ANGLE_AXIS;

    the_robot->EDP_data.get_arm_type = FRAME;
    //FRAME; //XYZ_EULER_ZYZ; //POSE_FORCE_TORQUE_AT_FRAME;
    the_robot->EDP_data.set_rmodel_type = TOOL_FRAME;
    the_robot->EDP_data.get_rmodel_type = TOOL_FRAME;
    //the_robot->EDP_data.motion_type = ABSOLUTE; //RELATIVE;
     the_robot->EDP_data.next_interpolation_type = EXTERNAL_INTERPOLATION_WITH_FORCE;
    the_robot->EDP_data.motion_type = ABSOLUTE; //PF_FIXED_FRAME_WITH_DESIRED_FORCE_OR_SPEED;

    the_robot->EDP_data.motion_steps = td.internode_step_no;
    the_robot->EDP_data.value_in_step_no = td.value_in_step_no;

    for(int i=0;i<6;i++)
    {
        the_robot->EDP_data.next_velocity[i] = 0;
        the_robot->EDP_data.next_force_xyz_torque_xyz[i] = 0;
        //	the_robot->EDP_data.MPselection_vector[i] = FORCE_SV_AX;
        //		 the_robot->EDP_data.MPselection_vector[i] = POSE_SV_AX;
    }



    for (int i=0;i<3;i++)
    {
        the_robot->EDP_data.next_inertia[i] = force_inertia_; // FORCE_INERTIA;
        the_robot->EDP_data.next_inertia[i+3] = torque_inertia_; //TORQUE_INERTIA;

        the_robot->EDP_data.next_reciprocal_damping[i] = force_reciprocal_damping_;
        the_robot->EDP_data.next_reciprocal_damping[i+3] = torque_reciprocal_damping_;
    }


    return true;
}
; // end: mp_vis_sac_lx_generator::first_step()

// ----------------------------------------------------------------------------------------------
// -----------------------------------  metoda	next_step -----------------------------------
// ----------------------------------------------------------------------------------------------

bool ecp_vis_sac_lx_generator::next_step ()
{
    // Generacja trajektorii prostoliniowej o zadany przyrost polozenia i orientacji
    // Funkcja zwraca false gdy koniec generacji trajektorii
    // Funkcja zwraca true gdy generacja trajektorii bedzie kontynuowana
    // UWAGA: dzialamy na jednoelementowej liscie robotow


    the_robot->EDP_data.set_type = ARM_DV;
    the_robot->EDP_data.instruction_type = SET_GET;
    //the_robot->EDP_data.get_type = NOTHING_DV;
    //the_robot->EDP_data.get_arm_type = INVALID_END_EFFECTOR;
    //the_robot->EDP_data.get_arm_type = INVALID_END_EFFECTOR;


    if(node_counter==1)
    {
        vsp_vis_sac->base_period=1;
        vsp_vis_sac->current_period=1; //1
    }

    if(node_counter==1)
    {
        std::cout  << std::endl << std::endl << std::endl << std::endl << std::endl;

        O_Tx_E.set_frame_tab(the_robot->EDP_data.current_arm_frame); // zarem
        std::cout << "YYY " << 	O_Tx_E  << std::endl;


        //O_Tx_E.set_frame_tab(the_robot->EDP_data.current_arm_frame); // zarem
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
        //	for (int i=0; i<6; i++)
        //		vsp_vis_sac->image.vis_sac.frame_E_r_G__f[i]=O_r_E[0][i]; //nie wiem czy potrzebne bo chyba  robot sie nie rusza

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
    			std::cout << vsp_vis_sac->image.vis_sac.frame_E_T_G[4*i+j];
    		}
    		std::cout << std::endl;
    	}
    */

    //std::cout << "C_T_G" << std::endl;
    //	for(int j=0; j<6; j++)
    //		{
    //			std::cout << vsp_vis_sac->image.vis_sac.frame_E_r_G__f[j] << ",";
    //		}
    //	std::cout << std::endl;





    C_Tx_G.set_xyz_rpy(vsp_vis_sac->image.vis_sac.frame_E_r_G__f[0],vsp_vis_sac->image.vis_sac.frame_E_r_G__f[1],
                       -vsp_vis_sac->image.vis_sac.frame_E_r_G__f[2],
                       vsp_vis_sac->image.vis_sac.frame_E_r_G__f[5]
                       ,0,0);
    //vsp_vis_sac->image.vis_sac.frame_E_r_G__f[5]);

    //std::cout << "MP C_T_G" << std::endl;
    //std::cout << C_Tx_G;

    //O_Tx_G.set_xyz_euler_zyz(0.950, 0.000, 0.265, 0.002, 1.481, 2.341);

    //	O_Tx_G.set_xyz_euler_zyz(0.950+0.058+vsp_vis_sac->image.vis_sac.frame_E_r_G__f[0],
    //	0.000+vsp_vis_sac->image.vis_sac.frame_E_r_G__f[1]-0.06,
    // 	0.265+vsp_vis_sac->image.vis_sac.frame_E_r_G__f[2]+0.900,
    //	0.002, 1.481+0.03, 2.341);

    //std::cout << "MP O_T_G pierwsze" << std::endl;
    //std::cout << O_Tx_G;

    C_Tx_G.get_xyz_rpy(C_r_G[0]);
    C2_Tx_G=C_Tx_G; //EIH

    std::cout << "ROZNICA:C " << fabs(C_r_G[1][0]-C_r_G[0][0])<<" "<< C_r_G[1][0] <<" " <<C_r_G[0][0] << std::endl;

    //podjazd gdy sie nie ruszamy
    if(fabs(O_r_G[1][0]-O_r_G[0][0])<=0.02 && fabs(O_r_G[1][1]-O_r_G[0][1])<=0.02) //0.007
    {
        steps2switch++;
        std::cout << "STOPPED" << std::endl;
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
        x2g+=0.0;  //0.002; //0.001; //NIE PODJEZDZAMY
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

    //x2g=-0.07;
    x2g=0;
    G_Tx_S.set_xyz_rpy(x2g, 0, 0, 0, 0, 0);

    //G_Tx_G2.set_xyz_euler_zyz( 0,0,0, 0.002, 1.481+0.03, 2.341);	//jesli chwytamy po przekatnej
    //G_Tx_G2.set_xyz_euler_zyz( 0,0,0, 0.002, 1.481+0.03, 3.141); // jesli chwytak na plasko
    G_Tx_G2.set_xyz_euler_zyz( 0,0,0, 1.564, 3.142, 0.000); //chwytak do dolu

    O_Tx_C.set_xyz_rpy( 0.009, //-0.09,
                        0.859,
                        0.345,
                        0,0,0);

    //std::cout << "MP O_T_C" << std::endl;
    //std::cout << O_Tx_C;
    //	std::cout << "MP C_T_G" << std::endl;
    //	std::cout << C_Tx_G;


    O_Tx_G=O_Tx_C*C_Tx_G;

    //std::cout << "MP O_T_G2" << std::endl;
    //std::cout << O_Tx_G;

    //std::cout << "MP O_T_G1" << std::endl;
    //std::cout << O_Tx_G;


    //skrot myslowy
    //	O_Tx_G=O_Tx_G*G_Tx_S;

    //std::cout << "MP O_T_G2" << std::endl;
    //std::cout << O_Tx_G;
    O_Tx_G=O_Tx_G*G_Tx_G2;
    //std::cout << "MP O_T_G3" << std::endl;
    //std::cout << O_Tx_G;


    //std::cout << "MP O_T_G" << std::endl;
    //std::cout << O_Tx_G;


    O_Tx_G.get_xyz_angle_axis(O_r_G[0]);

    O_Tx_E.set_frame_tab(the_robot->EDP_data.current_arm_frame);
    O_Tx_E.get_xyz_angle_axis(O_r_E[0]);


    O_Tx_EE.set_frame_tab(the_robot->EDP_data.current_arm_frame);

    //EIH
    O_Tx_G__C2=O_Tx_E*C2_Tx_G; //O_Tx_E*E_Tx_C2*C2_Tx_G;

    O_Tx_G__C2=(O_Tx_E*!G_Tx_G2)*C2_Tx_G*G_Tx_G2;

    O_Tx_G__C2.get_xyz_angle_axis(O_r_G[0]);

    std::cout << "MP O_T_G__C2" << std::endl;
    std::cout << O_Tx_G__C2;

    std::cout << "MP O_T_E" << std::endl;
    std::cout << O_Tx_E;

    std::cout << "MP C2_T_G" << std::endl;
    std::cout << C2_Tx_G;
    std::cout <<"Rota " << O_r_G[0][3] <<" " << O_r_G[0][4] << " "<< O_r_G[0][5] <<std::endl;
    //std::cout << "MP O_T_EE" << std::endl;
    //std::cout << O_Tx_EE;

    //skrot myslowy
    O_Tx_G__C2=(O_Tx_G__C2*!G_Tx_G2)*G_Tx_S*G_Tx_G2;

    std::cout << "MP O_T_G__C2" << std::endl;
    std::cout << O_Tx_G__C2;

    //	std::cout << "O_r_Ep[0][i]  O_r_Ep[1][i] O_eps_EG[0][i]   O_r_G[0][i] O_r_E[0][i]" << std::endl;
    std::cout << "poczatek ";
    std::cout << O_r_E[0][1] << " " <<  O_r_E[1][1] << std::endl;


    for (int i=0; i<6; i++)
    {
        O_eps_EG[0][i]=O_r_G[0][i]-O_r_E[0][i];
        //O_r_Ep[0][i]=O_r_E[0][i]+0.5*O_eps_EG[0][i]; //0.01
        O_r_Ep[0][i]=O_r_E[0][i]+gain[i]*O_eps_EG[0][i]; //0.01
        std::cout << "O_r_Ep[0][i]  O_r_E[0][i] O_eps_EG[0][i]  O_r_G[0][i] O_r_E[0][i]" << std::endl;

        std::cout << O_r_Ep[0][i]  << " " <<  O_r_E[0][i] << " " << O_eps_EG[0][i] << " " <<  O_r_G[0][i] << " " << O_r_E[0][i] << std::endl;
    }

    //O_eps_EG[0][2]=O_r_G[0][2]-O_r_E[0][2];
    //O_r_Ep[0][2]=O_r_E[0][2]+0.2*O_eps_EG[0][2];
    //O_r_Ep[0][2]=O_r_E[0][2];
    /*
    for (int i=3; i<6; i++)
{
    	O_eps_EG[0][i]=O_r_G[0][i]-O_r_E[0][i];
    	//O_r_Ep[0][i]=O_r_E[0][i]+0.5*O_eps_EG[0][i]; //0.01
    	O_r_Ep[0][i]=O_r_E[0][i]+gain[i]*O_eps_EG[0][i];
    //		std::cout << O_r_Ep[0][i]  << " " <<  O_r_E[0][i] << " " << O_eps_EG[0][i] << " " <<  O_r_G[0][i] << " " << O_r_E[0][i] << std::endl;
}
    */
    //O_r_Ep[0][0]=O_r_E[0][0]+0.005*O_eps_EG[0][0];
    /*
    O_r_Ep[0][2]=O_r_E[0][2]+0.01*O_eps_EG[0][2];
    O_r_Ep[0][3]=O_r_E[0][3]+0.1*O_eps_EG[0][3];
    */
    //O_r_Ep[0][0]=O_r_E[0][0];
    //O_r_Ep[0][1]=O_r_E[0][1];


    if(node_counter==1)
    {
        for (int i=0; i<6; i++)
        {
            O_r_Ep[0][i]=O_r_E[0][i];
        }
    }
    /*
    	for (int i=0; i<6; i++)
    	{
    		O_r_Ep[0][i]=O_r_E[0][i];
    	}
    */
    //O_r_Ep[0][4]=O_r_E[0][4]+0.002*O_eps_EG[0][4];
    //O_r_Ep[0][5]=O_r_E[0][5]+0.002*O_eps_EG[0][5];
    //

    /*
    std::cout << "przed kranc ";
     for (int i=0; i<6; i++)
    	{
    		std::cout << O_r_Ep[0][i] << " ";
    	}
     
    	std::cout << std::endl;
    */

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
    /*
    	std::cout << "po aaa ";
     for (int i=0; i<6; i++)
    	{
    		std::cout << O_r_Ep[0][i] << " ";
    	}
     
    		std::cout << std::endl;
     
    	std::cout << "po aaa2 ";
    		std::cout << O_r_Ep[0][1] << " " <<  O_r_Ep[1][1] << std::endl;
    */

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

    /*
    std::cout << "po vvv ";
     for (int i=0; i<6; i++)
    	{
    		std::cout << O_r_Ep[0][i] << " ";
    	}
    std::cout << std::endl;
    */

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

    /*
    	std::cout << "po kranc ";
     for (int i=0; i<6; i++)
    	{
    		std::cout << O_r_Ep[0][i] <<" ";
    	}
    */

    E_Tx_Ep = !O_Tx_E * O_Tx_Ep;


    E_Tx_Ep.get_xyz_angle_axis(E_r_Ep[0]);

    //O_Tx_Ep.get_frame_tab(the_robot->EDP_data.next_arm_frame); //jesli sie chcemy ruszyc

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
        C_r_G[1][i]=C_r_G[0][i];
    }



    //	O_Tx_Ep.get_frame_tab( the_robot->EDP_data.next_arm_frame); //jesli ABSOLUTNIE zadajemy O_Tx_E
    /*
    	if(node_counter==4)
    	{
    		vsp_vis_sac->base_period=1;
    		vsp_vis_sac->current_period=1; //1
    	}
     */

    //pose regulator
    /*
    for (int i=0; i<6; i++)
{
    	the_robot->EDP_data.next_XYZ_AA_arm_coordinates[i]=E_r_Ep[0][i];
}
     */


    //force regulator
    //sterujemy predkoscia
    /*
    for(int i=0;i<3; i++) {

    	the_robot->EDP_data.ECPtoEDP_position_velocity[i] = 0; //1.5*E_r_Ep[0][i];//*2*td.internode_step_no;
}

    for(int i=3;i<5;i++) {

    	the_robot->EDP_data.ECPtoEDP_position_velocity[i] = 0; //0.9*E_r_Ep[0][i];//*2*td.internode_step_no;
}

    for(int i=0;i<3; i++) {

    	the_robot->EDP_data.ECPtoEDP_position_velocity[i] = E_r_Ep[0][i]*
    	(double) (1/ ( ((double)STEP)*((double)step_no)*1) ); //1.5*E_r_Ep[0][i];//*2*td.internode_step_no;
}

    for(int i=3;i<5;i++) {

    	the_robot->EDP_data.ECPtoEDP_position_velocity[i] = E_r_Ep[0][i]*
    	(double) (1/ ( ((double)STEP)*((double)step_no)*1) );//*2*td.internode_step_no;
}
    */

    //sterujemy polozeniem


    for(int i=0;i<6;i++)
    {
        the_robot->EDP_data.next_velocity[i] = O_r_Ep[0][i];
    }
    /*
    for(int i=0;i<6;i++) {
    		 the_robot->EDP_data.ECPtoEDP_force_xyz_torque_xyz[i] = -E_r_Ep[0][i]*5000;
    		  the_robot->EDP_data.ECPtoEDP_force_xyz_torque_xyz[i] = -E_r_Ep[0][i]*100;
     
     
}
     
    */
    //	if(the_robot_con) irp6p->EDP_data.MPselection_vector[i] = FORCE_SV_AX;
    //	else
    // the_robot->EDP_data.MPselection_vector[i] = POSE_SV_AX;


    //	}


    the_robot->EDP_data.next_gripper_coordinate=the_robot->EDP_data.current_gripper_coordinate;

    /*******************************************************/



    /*

    // UWAGA: dzialamy na jednoelementowej liscie robotow
    if ( the_robot->EDP_data.ecp_reply == TASK_TERMINATED ) {
    	mp_msg->message("w mp task terminated");
    	return false;
} else return true;
    */

    return true;
}
; // end: bool tight_coop_generator::next_step ()
