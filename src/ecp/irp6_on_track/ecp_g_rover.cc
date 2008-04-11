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

ecp_vis_sac_lx_generator::ecp_vis_sac_lx_generator(ecp_task& _ecp_task, int step) :
	ecp_generator(_ecp_task)
{
	step_no = step;



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

	//SAC	
	O_weight__SAC=ecp_t.config.return_double_value("O_weight__SAC");
	O_gain__SAC[0]=ecp_t.config.return_double_value("O_gain0__SAC");
	O_gain__SAC[1]=ecp_t.config.return_double_value("O_gain1__SAC");
	O_gain__SAC[2]=ecp_t.config.return_double_value("O_gain2__SAC");
	O_gain__SAC[3]=ecp_t.config.return_double_value("O_gain3__SAC");
	O_gain__SAC[4]=ecp_t.config.return_double_value("O_gain4__SAC");
	O_gain__SAC[5]=ecp_t.config.return_double_value("O_gain5__SAC");
	
	C_weight__SAC=ecp_t.config.return_double_value("C_weight__SAC");
	C_gain__SAC[0]=ecp_t.config.return_double_value("C_gain0__SAC");
	C_gain__SAC[1]=ecp_t.config.return_double_value("C_gain1__SAC");
	C_gain__SAC[2]=ecp_t.config.return_double_value("C_gain2__SAC");
	C_gain__SAC[3]=ecp_t.config.return_double_value("C_gain3__SAC");
	C_gain__SAC[4]=ecp_t.config.return_double_value("C_gain4__SAC");
	C_gain__SAC[5]=ecp_t.config.return_double_value("C_gain5__SAC");
	
	f_weight__SAC=ecp_t.config.return_double_value("f_weight__SAC");
	f_gain__SAC[0]=ecp_t.config.return_double_value("f_gain0__SAC");
	f_gain__SAC[1]=ecp_t.config.return_double_value("f_gain1__SAC");
	f_gain__SAC[2]=ecp_t.config.return_double_value("f_gain2__SAC");
	f_gain__SAC[3]=ecp_t.config.return_double_value("f_gain3__SAC");
	f_gain__SAC[4]=ecp_t.config.return_double_value("f_gain4__SAC");
	f_gain__SAC[5]=ecp_t.config.return_double_value("f_gain5__SAC");

	//EIH
	C_weight__EIH=ecp_t.config.return_double_value("C_weight__EIH");
	C_gain__EIH[0]=ecp_t.config.return_double_value("C_gain0__EIH");
	C_gain__EIH[1]=ecp_t.config.return_double_value("C_gain1__EIH");
	C_gain__EIH[2]=ecp_t.config.return_double_value("C_gain2__EIH");
	C_gain__EIH[3]=ecp_t.config.return_double_value("C_gain3__EIH");
	C_gain__EIH[4]=ecp_t.config.return_double_value("C_gain4__EIH");
	C_gain__EIH[5]=ecp_t.config.return_double_value("C_gain5__EIH");
		
	f_weight__EIH=ecp_t.config.return_double_value("f_weight__EIH");
	f_gain__EIH[0]=ecp_t.config.return_double_value("f_gain0__EIH");
	f_gain__EIH[1]=ecp_t.config.return_double_value("f_gain1__EIH");
	f_gain__EIH[2]=ecp_t.config.return_double_value("f_gain2__EIH");
	f_gain__EIH[3]=ecp_t.config.return_double_value("f_gain3__EIH");
	f_gain__EIH[4]=ecp_t.config.return_double_value("f_gain4__EIH");
	f_gain__EIH[5]=ecp_t.config.return_double_value("f_gain5__EIH");


	force_inertia_=ecp_t.config.return_double_value("force_inertia_");
	torque_inertia_=ecp_t.config.return_double_value("torque_inertia_");
	force_reciprocal_damping_
			=ecp_t.config.return_double_value("force_reciprocal_damping_");
	torque_reciprocal_damping_
			=ecp_t.config.return_double_value("torque_reciprocal_damping_");

	x2g=ecp_t.config.return_double_value("x2g"); //x nibytoola

	x2g_begin=x2g;

	vis_phase = 0;
	steps2switch=0;

	phaseCEIH=0;
	
	CSAC_Tx_G_firstvalid=1;
	CEIH_Tx_G_firstvalid=1;

}
;

// ----------------------------------------------------------------------------------------------
// ---------------------------------    metoda	first_step -------------------------------------
// ----------------------------------------------------------------------------------------------

bool ecp_vis_sac_lx_generator::first_step()
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
	
	printf("KAMERA, KAMERA %d \n\n",vsp_vis_sac);

	idle_step_counter = 1;
	vsp_vis_sac->base_period=0; //1
	vsp_vis_sac->current_period=0; //MAC7
		printf("KAMERA, KAMERA2 \n\n");
	//vsp_force_irp6p->base_period=1;
	//td.interpolation_node_no = 1; //potrzebne? MAC7
	td.internode_step_no = 50; //step_no; 50
	td.value_in_step_no = td.internode_step_no - 5; //2 //10

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
	the_robot->EDP_data.next_interpolation_type= MIM; //TCIM
	the_robot->EDP_data.motion_type = ABSOLUTE; //PF_FIXED_FRAME_WITH_DESIRED_FORCE_OR_SPEED;

	the_robot->EDP_data.motion_steps = td.internode_step_no;
	the_robot->EDP_data.value_in_step_no = td.value_in_step_no;

	for (int i=0; i<6; i++)
	{
		the_robot->EDP_data.next_XYZ_AA_arm_coordinates[i] = 0;
		the_robot->EDP_data.next_force_xyz_torque_xyz[i] = 0;
		//	the_robot->EDP_data.MPselection_vector[i] = FORCE_SV_AX;
		//		 the_robot->EDP_data.MPselection_vector[i] = POSE_SV_AX;
	}

	for (int i=0; i<3; i++)
	{
		the_robot->EDP_data.next_inertia[i] = force_inertia_; // FORCE_INERTIA;
		the_robot->EDP_data.next_inertia[i+3] = torque_inertia_; //TORQUE_INERTIA;

		the_robot->EDP_data.next_reciprocal_damping[i]
				= force_reciprocal_damping_;
		the_robot->EDP_data.next_reciprocal_damping[i+3]
				= torque_reciprocal_damping_;
	}

	return true;
}
; // end: mp_vis_sac_lx_generator::first_step()

// ----------------------------------------------------------------------------------------------
// -----------------------------------  metoda	next_step -----------------------------------
// ----------------------------------------------------------------------------------------------

bool ecp_vis_sac_lx_generator::next_step()
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

		//G_Tx_G2.set_xyz_euler_zyz( 0,0,0, 0.002, 1.481+0.03, 2.341);	//jesli chwytamy po przekatnej
	//G_Tx_G2.set_xyz_euler_zyz( 0,0,0, 0.002, 1.481+0.03, 3.141); // jesli chwytak na plasko oryginal
	G_Tx_G2.set_xyz_euler_zyz( 0,0,0, 0.00, 1.57, 3.141); // jesli chwytak na plasko
	
	
	//G_Tx_G2.set_xyz_euler_zyz( 0,0,0, 1.564, 0.0, 0.000); //rover
	//G_Tx_G2.set_xyz_euler_zyz( 0,0,0,1.569070, 3.141593, 0.000000);


	if (node_counter==1)

	{
		vsp_vis_sac->base_period=1;
		vsp_vis_sac->current_period=1; //1
	}

	if (node_counter==1)
	{
		std::cout << std::endl << std::endl << std::endl << std::endl
				<< std::endl;

		O_Tx_E.set_frame_tab(the_robot->EDP_data.current_arm_frame); // zarem
		std::cout << "YYY " << O_Tx_E << std::endl;

		//O_Tx_E.set_frame_tab(the_robot->EDP_data.current_arm_frame); // zarem
		O_Tx_E.get_xyz_angle_axis(O_r_E[0]);
		O_Tx_E=O_Tx_E*!G_Tx_G2;

		std::cout << "MMM " << node_counter << std::endl;
		std::cout << "YYY " << O_Tx_E << std::endl;

		//	std::cout << O_Tx_E << std::endl;

		for (int i=0; i<6; i++)
		{

			O_r_Ep[0][i]=O_r_E[0][i];
			O_r_Ep[1][i]=O_r_E[0][i];
			O_r_Ep[2][i]=O_r_E[0][i];

			//O_r_Ep_d[i][1]=0;
			//O_r_Ep_d[i][2]=0;
			
			O_r_Ep_d[1][i]=0;
			O_r_Ep_d[1][i]=0;
			O_r_Ep_d[2][i]=0;
			
			CEIH_rpy_G[0][i]=0;
			CEIH_rpy_G[1][i]=0;
			CEIH_rpy_G[2][i]=0;
			
		}
		//	for (int i=0; i<6; i++)
		//		vsp_vis_sac->image.vis_sac.frame_E_r_G__f[i]=O_r_E[0][i]; //nie wiem czy potrzebne bo chyba  robot sie nie rusza



	

		std::cout << node_counter
				<<"------------------------------------------------------------------"
				<< std::endl;
	}

	


	//SAC
	CSAC_Tx_G.set_xyz_rpy(vsp_vis_sac->image.vis_sac.frame_E_r_G[0],
			vsp_vis_sac->image.vis_sac.frame_E_r_G[1],
			-vsp_vis_sac->image.vis_sac.frame_E_r_G[2],

			vsp_vis_sac->image.vis_sac.frame_E_r_G[5], 0, -0.05);
	//std::cout <<"Rota C_T_Gxxx " << vsp_vis_sac->image.vis_sac.frame_E_r_G[5] <<" " << O_r_G[0][4] << " "<< O_r_G[0][5] <<std::endl;


	//EIH
	//rover VSP_REPORT::
	//jesli byl odczyt z czujnika to przepisz odczytane wartosci - jesli nie przepisz stare 
	
	if(vsp_vis_sac->vsp_report_aux==VSP_REPLY_OK)
	{
		CEIH_rpy_G[0][0]=vsp_vis_sac->image.vis_sac.frame_E_r_G__f[0]-0.015; //-0.15
		CEIH_rpy_G[0][1]=vsp_vis_sac->image.vis_sac.frame_E_r_G__f[1]+0.03; //0.04
		CEIH_rpy_G[0][2]=-vsp_vis_sac->image.vis_sac.frame_E_r_G__f[2]+0.01; // kalib Y w O //+0.02
		CEIH_rpy_G[0][3]=0;
	 	CEIH_rpy_G[0][4]=0; 
	 	if(vsp_vis_sac->image.vis_sac.frame_E_r_G__f[5]<=0)
	 	{
			CEIH_rpy_G[0][5]=-vsp_vis_sac->image.vis_sac.frame_E_r_G__f[5]-0.800; //rover //0.8
		}
		else
		{
			CEIH_rpy_G[0][5]=-vsp_vis_sac->image.vis_sac.frame_E_r_G__f[5]+0.800; //rover //0.8
		}
	}
	else
	{
		for(int i=0; i<6; i++)
		{
			CEIH_rpy_G[0][i]=CEIH_rpy_G[1][i];
		}
		std::cout<<"NOTREADING " <<std::cout;
	}

	//w pierwszym kroku przychodza jakies smieci
	if (node_counter==1)
	{
		for(int i=0; i<6; i++)
		{
			CEIH_rpy_G[0][i]=0.0;
		}		
	}
	std::cout<<"RPY: ";
		
		for(int i=0; i<6; i++)
		{
			printf("%f ",CEIH_rpy_G[0][i]);
		}

	std::cout<<std::endl;

	CEIH_Tx_G.set_xyz_rpy(CEIH_rpy_G[0][0], CEIH_rpy_G[0][1], CEIH_rpy_G[0][2], CEIH_rpy_G[0][3], CEIH_rpy_G[0][4], CEIH_rpy_G[0][5]);
		

		
	/*
		CEIH_Tx_G.set_xyz_rpy(vsp_vis_sac->image.vis_sac.frame_E_r_G__f[0]-0.015, //-0.15
			vsp_vis_sac->image.vis_sac.frame_E_r_G__f[1]+0.03, //0.04
			-vsp_vis_sac->image.vis_sac.frame_E_r_G__f[2]+0.01, // kalib Y w O //+0.02
		//	-vsp_vis_sac->image.vis_sac.frame_E_r_G__f[3], 0, 0); //nomalnie
			//0, 0, -vsp_vis_sac->image.vis_sac.frame_E_r_G__f[5]-0.785000); //rover
			0, 0, -vsp_vis_sac->image.vis_sac.frame_E_r_G__f[5]-0.800); //rover //0.8
	*/
		
	
	
//vsp_vis_sac->vsp_report_aux==VSP_REPORT::VSP_REPLY_OK

//kostka	
/*
	CEIH_Tx_G.set_xyz_rpy(vsp_vis_sac->image.vis_sac.frame_E_r_G__f[0],
			vsp_vis_sac->image.vis_sac.frame_E_r_G__f[1],
			-vsp_vis_sac->image.vis_sac.frame_E_r_G__f[2], // kalib Y w O
			-vsp_vis_sac->image.vis_sac.frame_E_r_G__f[3], 0, 0); 	
*/


	CEIH_Tx_G__f.set_xyz_rpy(vsp_vis_sac->image.vis_sac.frame_E_r_G__CEIH[0],
			vsp_vis_sac->image.vis_sac.frame_E_r_G__CEIH[1],
			-vsp_vis_sac->image.vis_sac.frame_E_r_G__CEIH[2],
			-vsp_vis_sac->image.vis_sac.frame_E_r_G__CEIH[3], 0, 0); //-1

	for (int i=0; i<8; i++)
	{
		fEIH_G[i]=vsp_vis_sac->image.vis_sac.fEIH_G[i];
	}

	//podjazd gdy sie nie ruszamy
	if (fabs(O_r_G[1][0]-O_r_G[0][0])<=0.02 && fabs(O_r_G[1][1]-O_r_G[0][1])
			<=0.02) //0.007
	{
		steps2switch++;

	}
	else
	{
		x2g=x2g_begin;
		steps2switch=0;
	}

	if (steps2switch>15) // to approach 15
		vis_phase=1;

	if (vis_phase)
	{
		x2g+=0.0; //0.002; //0.001; //NIE PODJEZDZAMY
	}

	if (x2g>=0.0)
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

	G_Tx_S.set_xyz_rpy(x2g, 0, 0, 0, 0, 0);

	O_Tx_E.set_frame_tab(the_robot->EDP_data.current_arm_frame);
	O_Tx_E=O_Tx_E*!G_Tx_G2;
	O_Tx_E.get_xyz_angle_axis(O_r_E[0]);

	//SAC
	O_Tx_CSAC.set_xyz_rpy( 0.950+0.058, //-0.09,
			0.000-0.06, 0.265+0.900+0.075-0.105, 0, 0, 0);
	O_Tx_G__CSAC=O_Tx_CSAC*CSAC_Tx_G;
	O_Tx_G__CSAC=O_Tx_G__CSAC*G_Tx_S; //skrot myslowy
	O_Tx_G__CSAC.get_xyz_angle_axis(O_r_G__CSAC[0]);
	
	O_eps_EG__CSAC_norm=0.0;
	for (int i=0; i<6; i++)
		{
			O_eps_EG__CSAC[0][i]=(O_r_G__CSAC[0][i]-O_r_E[0][i]);
			O_eps_EG__CSAC_norm+=O_eps_EG__CSAC[0][i]*O_eps_EG__CSAC[0][i];
			O_eps_E__CSAC[0][i]=O_gain__SAC[i]*O_eps_EG__CSAC[0][i];		
		}	
	
//jesli nie widzi kostki bo jest za blisko zostaw stare namiary
	CEIH_Tx_G.get_xyz_angle_axis(CEIH_r_G[0]);
	CEIH_Tx_G__f.get_xyz_angle_axis(CEIH_r_G__f[0]);
	for (int i=0; i<6; i++)
	{
		std::cout << CEIH_r_G[0][i] << " ";
	}
/*	
	for (int i=0; i<6; i++)
	{
		std::cout << CEIH_r_G__f[0][i] << " ";
	}
*/
	//rover
//	std::cout << " ZZZ " << CEIH_r_G[0][0] << " " << -vsp_vis_sac->image.vis_sac.frame_E_r_G__f[5] << std::endl;

	if(CEIH_r_G[0][0]>0.12  
|| (CEIH_rpy_G[0][0]==0.0 && CEIH_rpy_G[0][1]==0.0 && CEIH_rpy_G[0][2]==0.0 &&
 CEIH_rpy_G[0][3]==0.0 && CEIH_rpy_G[0][4]==0.0 && CEIH_rpy_G[0][5]==0.0)) // || CEIH_Tx_G_firstvalid==0) //0.15
	{
	//EIH
	CEIH_Tx_G=CEIH_Tx_G*G_Tx_S;	
	O_Tx_G__CEIH=O_Tx_E*CEIH_Tx_G; //rota O_Tx_E 0,0,0 //E_TX_CEIH=1
	O_Tx_G__CEIH.get_xyz_angle_axis(O_r_G__CEIH[0]);
	
	}
	
	
/*	
std::cout << " O_Tx_G_CEIH ";
	std::cout << std::endl;
	std::cout << O_Tx_G__CEIH;
		std::cout << std::endl;
	std::cout << " O_r_G_CEIH ";
	for (int i=0; i<6; i++)
	{
		std::cout << O_r_G__CEIH[0][i]<< " ";
	}
	std::cout << std::endl;
	*/
//std::cout << " ZZZ " << CEIH_r_G[0][2] << std::endl;
//rover
//std::cout << " ZZZ " << O_r_E[0][2] << std::endl;
	//wyjdz
	if(O_r_E[0][2]<=0.012) //0.715
	{
		return false;
	}
	
//	O_rf_G__CEIH.set_values(O_r_G__CEIH[0]);
//	Ft_v_tr Jack(O_Tx_E,Ft_v_tr::V);
	
	
	//jak cos przyjdzie glupiego z CEIH
	//VSP_REPORT::VSP_REPLY_OK
	if (O_r_G__CEIH[0][0]>100 || O_r_G__CEIH[0][0]<-100)
	{
		for (int i=0; i<6; i++)
		{
			O_r_G__CEIH[0][i]=O_r_G__CEIH[1][i]; //EIH ONLY
		}
	}
	
	O_eps_EG__CEIH_norm=0.0;
	for (int i=0; i<6; i++)
		{
			O_eps_EG__CEIH[0][i]=(O_r_G__CEIH[0][i]-O_r_E[0][i]);
			O_eps_EG__CEIH_norm+=O_eps_EG__CEIH[0][i]*O_eps_EG__CEIH[0][i];
			O_eps_E__CEIH[0][i]=C_gain__EIH[i]*O_eps_EG__CEIH[0][i];		
		}
	
//printf("delta = %f %f %f", O_r_G__CEIH[0][0]-O_r_E[0][0], O_r_G__CEIH[0][1]-O_r_E[0][1], O_r_G__CEIH[0][2]-O_r_E[0][2]);

	//EIHJACK
	CEIH_Tx_G__f=CEIH_Tx_G__f*G_Tx_S;
	O_Tx_G__fEIH=O_Tx_E*CEIH_Tx_G__f; //rota O_Tx_E 0,0,0
	O_Tx_G__fEIH.get_xyz_angle_axis(O_r_G__fEIH[0]);
	
	O_eps_EG__fEIH_norm=0.0;
	for (int i=0; i<6; i++)
		{
			O_eps_EG__fEIH[0][i]=(O_r_G__fEIH[0][i]-O_r_E[0][i]);
			O_eps_EG__fEIH_norm+=O_eps_EG__fEIH[0][i]*O_eps_EG__fEIH[0][i];
			O_eps_E__fEIH[0][i]=f_gain__EIH[i]*O_eps_EG__fEIH[0][i];		
		}

CEIH_Tx_G.get_xyz_angle_axis(CEIH_r_G[0]);

//	std::cout << " O_T_E ";
	for (int i=0; i<6; i++)
	{
		std::cout << O_r_E[0][i] << " ";
	}
//	std::cout << std::endl;
/*	
	//std::cout << " SAC ";
	for (int i=0; i<6; i++)
	{
		std::cout << O_r_G__CSAC[0][i] << " ";
	}
	//std::cout << std::endl;
*/
	std::cout << " EIH ";
	for (int i=0; i<6; i++)
	{
		std::cout << O_r_G__CEIH[0][i]<< " ";
	}
//	std::cout << std::endl;
/*
	//std::cout << " EIH_JACK ";
	for (int i=0; i<6; i++)
	{
		std::cout << O_r_G__fEIH[0][i]<< " ";
	}
	//std::cout << std::endl;

	//std::cout << " EIH_FEAT ";
	for (int i=0; i<8; i++)
	{
		std::cout << fEIH_G[i]<< " ";
	}
	//std::cout << std::endl;

	std::cout <<  O_eps_EG__CSAC_norm << " ";
*/	
	std::cout << std::endl;
	

	//SWITCH
	/*
	O_eps_EG__CSAC_norm=0.0;
	for (int i=0; i<6; i++)
		{
			O_eps_EG__CSAC[0][i]=(O_r_G__CSAC[0][i]-O_r_E[0][i]);
			O_eps_EG__CSAC_norm+=O_eps_EG__CSAC[0][i]*O_eps_EG__CSAC[0][i];
		}	*/
/*
	O_eps_EG__CSAC_norm=0.0;
	for (int i=0; i<6; i++)
		{
			O_eps_EG__fEIH[0][i]=(O_r_G__CSAC[0][i]-O_r_E[0][i]);
			O_eps_EG__fEIH_norm+=O_eps_EG__CSAC[0][i]*O_eps_EG__CSAC[0][i];
		}	
*/
//std::cout << " EPS " << O_eps_EG__CSAC_norm << std::endl;

	//BOTH
	for (int i=0; i<6; i++)
	{
		//O_r_G[0][i]=0.5*O_r_G__CEIH[0][i]+0.5*O_r_G__CSAC[0][i]; //SAC+EIH
		O_r_G[0][i]=O_r_G__CSAC[0][i]; //SAC ONLY
		//O_r_G[0][i]=O_r_G__CEIH[0][i]; //EIH ONLY
		//O_r_G[0][i]=O_r_G__fEIH[0][i]; //EIH JACK ONLY
		
		//SWITCH
/*		
		if(O_eps_EG__CSAC_norm>=0.007 && phaseCEIH==0)
		{
			//O_r_G[0][i]=O_r_G__CSAC[0][i]; //SAC ONLY
			O_weight__SAC=1;
		}
		else
		{
			//O_r_G[0][i]=O_r_G__fEIH[0][i]; //EIH JACK ONLY	
			phaseCEIH=1;
			//O_r_G[0][i]=O_r_G__CEIH[0][i]; //EIH ONLY
			O_weight__SAC=0;
			C_weight__EIH=1;			
			
		}
*/
	}
	
		

	std::cout<<std::endl;


	for (int i=0; i<6; i++)
	{
	//	O_eps_EG[0][i]=O_r_G[0][i]-O_r_E[0][i];
	//	O_r_Ep[0][i]=O_r_E[0][i]+gain[i]*O_eps_EG[0][i]; //0.01
		//O_r_Ep[0][i]=O_r_E[0][i];//+O_weight__SAC*O_eps_E__CSAC[0][i]+C_weight__EIH*O_eps_E__CEIH[0][i]+f_weight__EIH*O_eps_E__fEIH[0][i];
		O_r_Ep[0][i]=O_r_E[0][i]+C_weight__EIH*O_eps_E__CEIH[0][i];
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

	//WAZNE - NIE RUSZAC SIE W 1SZYM KROKU
	if (node_counter==1)
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
	for (int i=0; i<6; i++)
		O_r_Ep_d[0][i]=O_r_Ep[0][i]-O_r_E[0][i];

	// roznica w 2 krokach -> docelowo przyspieszenie
	for (int i=0; i<6; i++)
		O_r_Ep_d2[0][i]=O_r_Ep_d[0][i]-O_r_Ep_d[1][i];

	//ograniczenie przyspieszenia (opoznienie zalatwia regulator proporcjonalny)
	for (int i=0; i<6; i++)
	{
		if ((fabs(O_r_Ep_d2[0][i])>=d2_u_max[i]) && (fabs(O_r_Ep_d[0][i])
				>=fabs(O_r_Ep_d[1][i])))
		{
			if (O_r_Ep_d[0][i]>=0)
			{
				O_r_Ep_d[0][i]=O_r_Ep_d[1][i]+d2_u_max[i];
				O_r_Ep[0][i]=O_r_E[0][i]+O_r_Ep_d[0][i];
			}
			else if (O_r_Ep_d[0][i]<0)
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
	for (int i=0; i<6; i++)
	{
		if (O_r_Ep_d[0][i]>=d_u_max[i])
		{
			O_r_Ep[0][i]=O_r_E[0][i]+d_u_max[i];

		}

		if (O_r_Ep_d[0][i]<=-1*d_u_max[i])
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
	for (int i=0; i<6; i++)
	{
		O_r_Ep_d2[0][i]=O_r_Ep_d[0][i]-O_r_Ep_d[1][i];
	}

	//krancowki na sterowaniu - nie wiemy jakie krancowki dla angle axis
	for (int i=0; i<3; i++)
	{
		if (O_r_Ep[0][i]<measure_border_d[i])
			O_r_Ep[0][i]=measure_border_d[i];

		if (O_r_Ep[0][i]>measure_border_u[i])
			O_r_Ep[0][i]=measure_border_u[i];
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

	//	E_Tx_Ep = !O_Tx_E * O_Tx_Ep;


	//	E_Tx_Ep.get_xyz_angle_axis(E_r_Ep[0]);

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
//		O_r_Ep_d[i][2]=O_r_Ep_d[i][1];
//		O_r_Ep_d[i][1]=O_r_Ep_d[i][0];

		O_r_Ep_d[2][i]=O_r_Ep_d[2][i];
		O_r_Ep_d[1][i]=O_r_Ep_d[1][i];

		O_r_G[1][i]=O_r_G[0][i];
		C_r_G[1][i]=C_r_G[0][i];
		
		
		CEIH_r_G[1][i]=CEIH_r_G[0][i];
		CEIH_r_G__f[1][i]=CEIH_r_G__f[0][i];
		CEIH_rpy_G[1][i]=CEIH_rpy_G[1][i];
		
	}



	//zmiana roty
	//O_Tx_E=O_Tx_E*G_Tx_G2;
	O_Tx_Ep=O_Tx_Ep*G_Tx_G2;

	O_Tx_Ep.get_xyz_angle_axis(O_r_Ep[0]);




	for (int i=0; i<6; i++)
	{
		the_robot->EDP_data.next_XYZ_AA_arm_coordinates[i] = O_r_Ep[0][i];
	}



	the_robot->EDP_data.next_gripper_coordinate
			=the_robot->EDP_data.current_gripper_coordinate;


	return true;
}
; // end: bool tight_coop_generator::next_step ()
