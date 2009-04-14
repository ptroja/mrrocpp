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
#include "mp/mp_g_vis_sac.h"

namespace mrrocpp {
namespace mp {
namespace generator {

//globalne do wywalenia

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

//double measure_border_u[]={0.840, 0.150, 0.305, 0.606, 1.57, 3.12}; //Zmienic ogranicz Z
//double measure_border_d[]={0.570, -0.150, 0.185, -0.606, 1.267, 1.8}; //by na Z 0.078 gamma 2.237 //byla beta 1.32

double measure_border_u[]={1.090, 0.150, 0.305, 0.606, 1.57, 3.12}; //Zmienic ogranicz Z
double measure_border_d[]={0.82, -0.150, 0.155, -0.606, 1.267, 1.8}; //by na Z 0.078 gamma 2.237 //byla beta 1.32


//double measure_border_u[]={1.000, 0.220, 0.338, 0.606, 1.57, 3.12}; //Zmienic ogranicz Z
//double measure_border_d[]={0.810, -0.250, 0.130, -0.606, 1.267, 1.8}; //by na Z 0.078 gamma 2.237 //byla beta 1.32


//double d_u_max[6]={0.005, 0.0125, 0.005, 0.01, 0.005, 0.01};
//double d2_u_max[6]={0.001, 0.001, 0.001, 0.01, 0.005, 0.005}; //tylko ustalone dla Y

double d_u_max[6]={0.010, 0.0250, 0.010, 0.02, 0.010, 0.04}; //td.internode_step_no = 80;
double d2_u_max[6]={0.002, 0.002, 0.002, 0.02, 0.01, 0.01}; //tylko ustalone dla Y

int valid_measure;

double x2g=-0.1; //x nibytoola

int vis_phase = 0;

int steps2switch=0;

double aux=0;
double aux2=0;


#define PRINTB 1



    vis_sac::vis_sac(task::base& _mp_task, int step): base (_mp_task), irp6ot_con(1), irp6p_con(1){ 
        step_no = step;          
    };  
	


// ----------------------------------------------------------------------------------------------
// ---------------------------------    metoda	first_step -------------------------------------
// ----------------------------------------------------------------------------------------------

bool vis_sac::first_step () {
  // Generacja trajektorii prostoliniowej o zadany przyrost polozenia i oreintacji
  // Funkcja zwraca false gdy koniec generacji trajektorii
  // Funkcja zwraca true gdy generacja trajektorii bedzie kontynuowana
  printf("EYE GEN XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX\n");
  
	for (int i=0; i<6; i++)
  		for (int j=0; j<5; j++)
			measure[i][j]=0;
	
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
	irp6ot->ecp_td.next_tool_frame[0][0]=1; irp6ot->ecp_td.next_tool_frame[1][0]=0; 
	irp6ot->ecp_td.next_tool_frame[2][0]=0; irp6ot->ecp_td.next_tool_frame[0][0]=0;

	irp6ot->ecp_td.next_tool_frame[0][1]=0; irp6ot->ecp_td.next_tool_frame[1][1]=1;
	irp6ot->ecp_td.next_tool_frame[2][1]=0; irp6ot->ecp_td.next_tool_frame[1][3]=0;

	irp6ot->ecp_td.next_tool_frame[0][2]=0; irp6ot->ecp_td.next_tool_frame[1][2]=0;
	irp6ot->ecp_td.next_tool_frame[2][2]=1; irp6ot->ecp_td.next_tool_frame[2][3]=0.25;

	
	irp6ot->ecp_td.mp_command = lib::NEXT_POSE; 
	irp6ot->ecp_td.instruction_type = lib::SET_GET; //GET;
	irp6ot->ecp_td.get_type = RMODEL_DV; //ARM_DV;
	irp6ot->ecp_td.set_type = RMODEL_DV; //ARM_DV;
	irp6ot->ecp_td.set_arm_type = lib::XYZ_EULER_ZYZ; //POSE_FORCE_TORQUE_AT_FRAME;
	irp6ot->ecp_td.get_arm_type = lib::XYZ_EULER_ZYZ; //POSE_FORCE_TORQUE_AT_FRAME;
	irp6ot->ecp_td.set_rmodel_type = lib::TOOL_FRAME;
	irp6ot->ecp_td.get_rmodel_type = lib::TOOL_FRAME;
	irp6ot->ecp_td.motion_type = lib::ABSOLUTE; //RELATIVE;
			irp6ot->ecp_td.next_interpolation_type = lib::MIM;
	irp6ot->ecp_td.motion_steps = td.internode_step_no;
	irp6ot->ecp_td.value_in_step_no = td.value_in_step_no;
	
	
	return true;
}; // end: vis_sac::first_step()

// ----------------------------------------------------------------------------------------------
// -----------------------------------  metoda	next_step -----------------------------------
// ----------------------------------------------------------------------------------------------

bool vis_sac::next_step () {
 // Generacja trajektorii prostoliniowej o zadany przyrost polozenia i orientacji
 // Funkcja zwraca false gdy koniec generacji trajektorii
 // Funkcja zwraca true gdy generacja trajektorii bedzie kontynuowana
 // UWAGA: dzialamy na jednoelementowej liscie robotow
 	int i=0;
	
	
	if (check_and_null_trigger()) return false;
 	
	if(node_counter==1 || node_counter==2)
	{
		irp6ot->ecp_td.instruction_type = lib::GET;
		irp6ot->ecp_td.get_type = ARM_DV | RMODEL_DV;

		// UWAGA: dzialamy na jednoelementowej liscie robotow
		if ( irp6ot->ecp_td.ecp_reply == lib::TASK_TERMINATED ) {
			sr_ecp_msg.message("w mp task terminated");
			return false;
		} else return true;
		
    		return true;
	}

	if(node_counter!=1 && node_counter!=2)
	{
	 	irp6ot->ecp_td.set_type = ARM_DV;
		irp6ot->ecp_td.instruction_type = lib::SET;
	  	irp6ot->ecp_td.get_type = NOTHING_DV;
	 	irp6ot->ecp_td.get_arm_type = lib::INVALID_END_EFFECTOR;
	} 
	
	
	if(node_counter==3 || node_counter==4)
	{
		for (i=0; i<6; i++)
		{	
				
			pose[i][0]=irp6ot->ecp_td.current_XYZ_ZYZ_arm_coordinates[i];
			pose[i][1]=irp6ot->ecp_td.current_XYZ_ZYZ_arm_coordinates[i];
			pose[i][2]=irp6ot->ecp_td.current_XYZ_ZYZ_arm_coordinates[i];
			measure[i][0]=0; //irp6ot->ecp_td.current_XYZ_ZYZ_arm_coordinates[i]; //GDY NIE MA INFO Z KAM
			measure[i][1]=0;  //irp6ot->ecp_td.current_XYZ_ZYZ_arm_coordinates[i];
			measure[i][2]=0; //irp6ot->ecp_td.current_XYZ_ZYZ_arm_coordinates[i];
			stearing[i][1]=irp6ot->ecp_td.current_XYZ_ZYZ_arm_coordinates[i];
			stearing[i][2]=irp6ot->ecp_td.current_XYZ_ZYZ_arm_coordinates[i];
			stearing_d[i][1]=0;
			stearing_d[i][2]=0;
			lasty=1000*(measure[i][0]-0.934); //proteza y skacze o 10mm
		}	
	}

#ifdef PRINT
	printf("%d CRR =", node_counter);
	for (i=0; i<6; i++)
	{	
	
	printf("%f ",irp6ot->ecp_td.current_XYZ_ZYZ_arm_coordinates[i]);
		
	}
	printf("\n");
#endif
	
	std::cout << "GGGGGG" << std::endl;
	
	for(int i=0; i<4; i++)
	{
		for(int j=0; j<4; j++)
		{
			frame1[i][j]=vsp_vis_sac->image.sensor_union.vis_sac.frame_E_T_G[4*i+j];
		}
	}
	
	for(int i=0; i<6; i++)
		vec1[i]=0.001*vsp_vis_sac->image.sensor_union.vis_sac.frame_E_r_G__f[i];
	
	std::cout << "rrrrrrrrrrr:" <<std::endl;
	for(int i=0; i<6; i++)
		std::cout << vec1[i] << std::endl;
	
	std::cout << std::endl;
	//tak dziwnie, bo inaczej 3 ostatnie pozycje 1szego wiersza E_Tx_G sa zerami 
	for(int i=3; i>=0; i--)
	{
		for(int j=3; j>=0; j--)
		{
			//frame1[i][j]=vsp_vis_sac->image.sensor_union.camera.frame[4*i+j];
			//aux=vsp_vis_sac->image.sensor_union.camera.frame[4*i+j];
			E_Tx_G.set_value(j,i,frame1[i][j]);
			E_Tx_G.get_value(j,i,aux);
			E_Tx_G.get_value(3,0,aux2);
			//std::cout << aux << " " << aux2 << " | ";
		}
	//	std::cout << std::endl;
	}
	
	//std::cout << E_Tx_G;
	
	//valid_measure=(int)frame1[3][3];
	//E_Tx_G.get_value(3,3,aux);
	//valid_measure=(int)aux;	


E_Tx_G.get_value(3,0,aux);
E_Tx_G.get_value(3,1,aux2);

E_Tx_G.set_value(3,1,0.001*aux);
E_Tx_G.set_value(3,0,0.001*aux2);

E_Tx_G.get_value(3,2,aux);
E_Tx_G.set_value(3,2,-0.001*aux);


#ifdef PRINT		
std::cout << "ECL" << std::endl;
	for(int i=0; i<4; i++)
	{
		for(int j=0; j<4; j++)
			std::cout << frame1[i][j] <<' ';
		std::cout <<std::endl;
	}
	//E_Tx_G.set_value(3,0,0.7);
	//E_Tx_G.get_value(1,3,aux2);
	 
	E_Tx_G.get_value(3,0,aux);
	E_Tx_G.get_value(3,1,aux2);
	std::cout << aux << " u " << aux2 <<std::endl;
	std::cout << E_Tx_G;
#endif

/*
//Proteza bo y w VSP skacze o 10mm raz na jakis czas
if(frame1[1][3]>lasty+8 && frame1[1][3]<lasty+12)
	frame1[1][3]=lasty;
else
	lasty=frame1[1][3];

//gdy zaslania chwytak
	if(x2g>-0.08)
	{
		for(int i=0; i<4; i++)
		for(int j=0; j<4; j++)
			frame1[i][j]=lastframe[i][j];
	}
	
		for(int i=0; i<4; i++)
		for(int j=0; j<4; j++)
			lastframe[i][j]=frame1[i][j];

	//UWAGA - ZAMIANA OSI X I Y
	if(node_counter!=3 && node_counter!=4)
	{
		//^{0}T_{G}=^{0}T_{C}*^{C}T_{G}
		measure[0][0]=0.001*frame1[1][3]+0.754-0.09+0.07-0.06+0.25; //-0.10
		measure[1][0]=0.001*frame1[0][3]-0.024+0.004+0.017;
		measure[2][0]=-0.001*frame1[2][3]+1.127-0.03+0.02-0.001+0.025;  // -0.035 na srodek 
		frame1[0][3]=measure[0][0];
		frame1[1][3]=measure[1][0];
		frame1[2][3]=measure[2][0];
		
		//printf("MES1 %f, %f %f\n",measure[0][0], measure[1][0], measure[2][0]);  
		//^{0}T_{P}=^{0}T_{G}*^{G}T_{P} //0.1 na Y - korekta kinemtyczna
		measure[0][0]=frame1[1][0]*(x2g)+frame1[1][1]*(0)+frame1[1][2]*0+frame1[0][3];
		measure[1][0]=frame1[0][0]*(x2g)+frame1[0][1]*(0)+frame1[0][2]*0+frame1[1][3];
		measure[2][0]=frame1[2][0]*(x2g)+frame1[2][1]*0+frame1[2][2]*0+frame1[2][3];
		
		//printf("MES2 %f, %f %f\n",measure[0][0], measure[1][0], measure[2][0]);  
		//measure[0][0]=frame1[1][0]*(-0.25)+frame1[1][0]*(0)+frame1[1][2]*0+frame1[0][3];
		//measure[1][0]=frame1[0][0]*(-0.25)+frame1[0][0]*(0)+frame1[0][2]*0+frame1[1][3];
		//measure[2][0]=frame1[2][1]*(-0.25)+frame1[2][0]*0+frame1[2][2]*0+frame1[2][3];
		//printf("%f %f %f\n",frame1[1][0]*(0.1)+frame1[1][1]*0+frame1[1][2]*0,frame1[1][3],measure[1][0]);
	}
*/
	
	E_Tx_G.get_xyz_euler_zyz(measure_aux);
	
	std::cout << "r222222222:" <<std::endl;
	for(int i=0; i<6; i++)
		std::cout << measure_aux[i] << std::endl;
	
	for(int i=0; i<6; i++)
	{
		measure[i][0]=measure_aux[i];
	}
	
	measure[0][0]=vec1[0];
	measure[1][0]=vec1[1];
	
#ifdef PRINTB	
	std::cout << "ECL ErG" << std::endl;
	for(int i=0; i<6; i++)
	{
		std::cout << measure[i][0] <<' ';
	}
	std::cout <<std::endl;
#endif
	
/*	
	//podjazd gdy sie nie ruszamy
	if(fabs(measure[0][1]-measure[0][0])<=0.01 && fabs(measure[1][1]-measure[1][0])<=0.01) //0.007
	{
		steps2switch++;
	}
	else
	{
		steps2switch=0;
	}
	

	if(steps2switch>15) // to approach 15
		vis_phase=1;

	
	if(vis_phase)
	{
		x2g+=0.004; //0.001;
	}
	
	if(x2g>0.020) 
	{
		x2g=0.02;

		//return true; //to avoid graspin'
		return false;	//to grasp

	}
	
	
	//odjazd gdy sie ruszymy
	if(fabs(measure[0][1]-measure[0][0])>=0.050 || fabs(measure[1][1]-measure[1][0])>=0.050 || fabs(measure[0][1]-measure[0][0])>=0.050)
	{
		x2g=-0.1;
		vis_phase=0;
		steps2switch=0;
	}
	
	gammax=atan2(frame1[2][1],frame1[2][2]);
	beta=atan2(-frame1[2][0],sqrt(frame1[0][0]*frame1[0][0]+frame1[1][0]*frame1[1][0]));
	alfa=atan2(frame1[1][0],frame1[0][0]);
	
	
	
	alfa-=M_PI/2;
	alfa*=-1;
	
	
#ifdef PRINT	
		std::cout <<"alfa="<<180*alfa/M_PI<<"     ";
#endif 	

	if(beta<0) beta*=-1;
	
	if(gammax>0.5) gammax=0.5;
	
#ifdef PRINT
	std::cout <<"alfa="<<180*alfa/M_PI<<"     " <<alfa<<" alfa dobra =" << 180*atan2(frame1[0][0],frame1[1][0])/M_PI << std::endl;
	std::cout <<"beta="<<180*beta/M_PI<<"     " <<beta<<std::endl;
	std::cout <<"gamma="<<180*gammax/M_PI<<"     " <<gammax<<std::endl;
#endif 
	
	if(node_counter!=3 && node_counter!=4)
	{
		//measure[1][0]=0.140;
		measure[3][0]=alfa;
		measure[4][0]=1.57-beta;
		measure[5][0]=3.14+gammax-0.785;
	}
	
#ifdef PRINT 
	printf("%d MSR =", node_counter);
	for (i=0; i<6; i++)
	{	
		printf("%f ",measure[i][0]);	
	}
	printf("\n");
#endif
	
	// krancowki
	for(int i=0; i<6; i++)
	 {
	 	 if(measure[i][0]<measure_border_d[i]) 
	 		measure[i][0]=measure_border_d[i];
	 
	 	if(measure[i][0]>measure_border_u[i]) 
	 		measure[i][0]=measure_border_u[i];
	 }
	 
#ifdef PRINT 
	printf("%d MSQ =", node_counter);
	for (i=0; i<6; i++)
	{	
		printf("%f ",measure[i][0]);	
	}
	printf("\n");
#endif
*/

/*
	for(int i=0; i<6; i++)
		stearing[i][0]=stearing[i][1] + 0.5*(measure[i][0]-stearing[i][1]);
	
	stearing[2][0]=stearing[2][1] + 0.2*(measure[2][0]-stearing[2][1]);
	stearing[3][0]=stearing[3][1] + 0.1*(measure[3][0]-stearing[3][1]);
	stearing[4][0]=stearing[4][1] + 0.05*(measure[4][0]-stearing[4][1]);
	stearing[5][0]=stearing[5][1] + 0.07*(measure[5][0]-stearing[5][1]); //0.05
*/
/*	
	for(int i=0; i<6; i++)
	{
		stearing[i][0]=stearing[i][1] + 0.5*(measure[i][0]);
	}
*/
	stearing[0][0]=stearing[0][1] + 0.1*(measure[0][0]-0.15);
	stearing[1][0]=stearing[1][1] + 0.1*(measure[1][0]);
	stearing[2][0]=stearing[2][1] + 0.05*(measure[2][0]);
	stearing[3][0]=stearing[3][1] - 0.05*(measure[3][0]);
	stearing[4][0]=stearing[4][1] + 0.05*(measure[4][0]);
	stearing[5][0]=stearing[5][1] + 0.07*(measure[5][0]); //0.05

//stearing[i][0]=stearing[i][0]-0.158; //manianiasty niby tool

#ifdef PRINTB
	printf("%d STR =", node_counter);
	for (i=0; i<6; i++)
	{	
		printf("%f ",stearing[i][0]);
	}
	printf("\n");
#endif 

#ifdef PRINTB
	printf("%d STR =", node_counter);
	for (i=0; i<6; i++)
	{	
		printf("%f ",stearing[i][0]);
	}
	printf("\n");
#endif 


	 
	// roznica w kroku -> docelowo predkosc
	for(int i=0; i<6; i++)
		stearing_d[i][0]=stearing[i][0]-stearing[i][1];
	 	
	// roznica w 2 krokach -> docelowo przyspieszenie
	for(int i=0; i<6; i++)
		stearing_d2[i][0]=stearing_d[i][0]-stearing_d[i][1];
 
 
	//ograniczenie przyspieszenia (opoznienie zalatwia regulator proporcjonalny) 
	for(int i=0; i<6; i++)
	{
	 	if((fabs(stearing_d2[i][0])>=d2_u_max[i]) && (fabs(stearing_d[i][0])>=fabs(stearing_d[i][1])) )
	 	{
	 		if(stearing_d[i][0]>=0)
	 		{
	 			stearing_d[i][0]=stearing_d[i][1]+d2_u_max[i];	
	 			stearing[i][0]=stearing[i][1]+stearing_d[i][0];
	 		}	
	 		else
			if(stearing_d[i][0]<0)
	 		{
	 			stearing_d[i][0]=stearing_d[i][1]-d2_u_max[i];	
	 			stearing[i][0]=stearing[i][1]+stearing_d[i][0];
	 		}		
		} 
	}


	 //ograniczenie predkosci
	 for(int i=0; i<6; i++)
	 {
	 		if(stearing_d[i][0]>=d_u_max[i])
	 		{
	 			stearing[i][0]=stearing[i][1]+d_u_max[i];
	 	
	 		}
			
	 		if(stearing_d[i][0]<=-1*d_u_max[i])
	 		{
	 			stearing[i][0]=stearing[i][1]-d_u_max[i];
	 	
	 		}		
	 }
 
	//ograniczenie przyspieszenia
	for(int i=0; i<6; i++)
	{ 
		stearing_d2[i][0]=stearing_d[i][0]-stearing_d[i][1];
	} 


#ifdef PRINTB
	printf("%d STQ =", node_counter);
	for (i=0; i<6; i++)
	{	
		printf("%f ",stearing[i][0]);	
	}
	printf("\n");
#endif


	//krancowki na sterowaniu 
	for(int i=0; i<6; i++)
	{
	 	if(stearing[i][0]<measure_border_d[i]) 
	 		stearing[i][0]=measure_border_d[i];
	 
	 	if(stearing[i][0]>measure_border_u[i]) 
	 		stearing[i][0]=measure_border_u[i];
	}
 

#ifdef PRINTB
	 printf("%d STR0=", node_counter);
	 for (int i=0; i<6 ; i++)
	{            
	       printf("%f  ",stearing[i][0]);
	}
	printf("\n");
#endif
	
#ifdef PRINTB
	printf("%d STR1=", node_counter);
	for (int i=0; i<6 ; i++)
	{  
	       printf("%f  ",stearing[i][1]);
	}
	printf("\n");
#endif 
	
	if(node_counter==4)
	{
		vsp_vis_sac->base_period=1;
		vsp_vis_sac->current_period=1;
	}

     for (i=0; i<6; i++)
	{
		irp6ot->ecp_td.next_XYZ_ZYZ_arm_coordinates[i]=stearing[i][0];
		
		stearing[i][2]=stearing[i][1];
		stearing[i][1]=stearing[i][0];
		stearing_d[i][2]=stearing_d[i][1];
		stearing_d[i][1]=stearing_d[i][0];
		measure[i][2]=measure[i][1];
		measure[i][1]=measure[i][0];
	}
	irp6ot->ecp_td.next_gripper_coordinate=irp6ot->ecp_td.current_gripper_coordinate;
	
	/*WYLACZENIE NIESTEROWANYCH OSI*/
	for (i=6; i<6; i++)
      {
      	irp6ot->ecp_td.next_XYZ_ZYZ_arm_coordinates[i]=irp6ot->ecp_td.current_XYZ_ZYZ_arm_coordinates[i];
      }
      for (i=0; i<6; i++)
      {
      	irp6ot->ecp_td.next_XYZ_ZYZ_arm_coordinates[i]=irp6ot->ecp_td.current_XYZ_ZYZ_arm_coordinates[i];
      }
	/*******************************************************/
	
#ifdef PRINTA	
	printf("%d ARM= ", node_counter);
 	for (int i=0; i<6 ; i++)             
		printf("%f  ",irp6ot->ecp_td.next_XYZ_ZYZ_arm_coordinates[i]);
	printf("\n");	
#endif


	// UWAGA: dzialamy na jednoelementowej liscie robotow
	if ( irp6ot->ecp_td.ecp_reply == lib::TASK_TERMINATED ) {
		sr_ecp_msg.message("w mp task terminated");
		return false;
	} else return true;
}; // end: bool tight_coop_generator::next_step ()


} // namespace generator
} // namespace mp
} // namespace mrrocpp
