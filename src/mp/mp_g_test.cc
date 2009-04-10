#include <stdio.h>
#include <math.h>
#include <iostream>

#include "common/typedefs.h"
#include "common/impconst.h"
#include "common/com_buf.h"

#include "lib/srlib.h"
#include "mp/mp.h"
#include "mp/mp_g_test.h"


namespace mrrocpp {
namespace mp {
namespace generator {


// extern ProxyPids MPProxyPids; // by Y&W zamienione na pulsy

	double measure_border_u[]={0.903, 0.800, 1.026, -0.044, 0, 0};
	double measure_border_d[]={0.705, -0.800, 0.919, -0.406, 0, 0};
	
	double d_u_max[6]={0.0005, 0.005, 0.0005, 0.0005, 0.0005, 0.0005};
	
	int the_first_big=1;

	double coordinate_delta2[6];



	MP_vf_generator::MP_vf_generator(task::mp_task& _mp_task, int step): mp_generator (_mp_task){ 
		step_no = step;          
	};  

// ----------------------------------------------------------------------------------------------
// ---------------------------------    metoda	first_step -------------------------------------
// ----------------------------------------------------------------------------------------------

bool MP_vf_generator::first_step () {
  // Generacja trajektorii prostoliniowej o zadany przyrost polozenia i oreintacji
  // Funkcja zwraca false gdy koniec generacji trajektorii
  // Funkcja zwraca true gdy generacja trajektorii bedzie kontynuowana
  
  
  
  
      idle_step_counter = 1;
  
// 	double tmp_vector[3];

	for (int i=0; i<6; i++)
		delta[i]=0.0;
	
	the_first=1;
	the_second=0;

	for (int i=0; i<6; i++)
 		for (int j=0; j<5; j++)
			measure[i][j]=0;
			
			
    td.internode_step_no = step_no;
     td.value_in_step_no = td.internode_step_no - 2;
    
        // track
	robot_m[ROBOT_IRP6_ON_TRACK]->ecp_td.mp_command = NEXT_POSE; 
	robot_m[ROBOT_IRP6_ON_TRACK]->ecp_td.instruction_type = GET;
	robot_m[ROBOT_IRP6_ON_TRACK]->ecp_td.get_type = ARM_DV;
	robot_m[ROBOT_IRP6_ON_TRACK]->ecp_td.set_type = ARM_DV;
	
	robot_m[ROBOT_IRP6_ON_TRACK]->ecp_td.set_arm_type = JOINT;
	robot_m[ROBOT_IRP6_ON_TRACK]->ecp_td.get_arm_type = JOINT;
	robot_m[ROBOT_IRP6_ON_TRACK]->ecp_td.motion_type = ABSOLUTE;
	robot_m[ROBOT_IRP6_ON_TRACK]->ecp_td.next_interpolation_type = MIM;
	robot_m[ROBOT_IRP6_ON_TRACK]->ecp_td.motion_steps = td.internode_step_no;
	robot_m[ROBOT_IRP6_ON_TRACK]->ecp_td.value_in_step_no = td.value_in_step_no;
    
    
        // postument
 	robot_m[ROBOT_IRP6_POSTUMENT]->ecp_td.mp_command = NEXT_POSE; 
	robot_m[ROBOT_IRP6_POSTUMENT]->ecp_td.instruction_type = GET;
	robot_m[ROBOT_IRP6_POSTUMENT]->ecp_td.get_type = ARM_DV;
	robot_m[ROBOT_IRP6_POSTUMENT]->ecp_td.set_type = ARM_DV;
	
	robot_m[ROBOT_IRP6_POSTUMENT]->ecp_td.set_arm_type = JOINT;
	robot_m[ROBOT_IRP6_POSTUMENT]->ecp_td.get_arm_type = JOINT;
	robot_m[ROBOT_IRP6_POSTUMENT]->ecp_td.motion_type = ABSOLUTE;
	robot_m[ROBOT_IRP6_POSTUMENT]->ecp_td.next_interpolation_type = MIM;
	robot_m[ROBOT_IRP6_POSTUMENT]->ecp_td.motion_steps = td.internode_step_no;
	robot_m[ROBOT_IRP6_POSTUMENT]->ecp_td.value_in_step_no = td.value_in_step_no;
 
/*   
    // speaker
    	robot_list->next->next->E_ptr->ecp_td.mp_command = NEXT_POSE; 
	robot_list->next->next->E_ptr->ecp_td.instruction_type = GET;
	robot_list->next->next->E_ptr->ecp_td.get_type = ARM_DV;
	robot_list->next->next->E_ptr->ecp_td.set_type = ARM_DV;
	
	robot_list->next->next->E_ptr->ecp_td.set_arm_type = XYZ_EULER_ZYZ;
	robot_list->next->next->E_ptr->ecp_td.get_arm_type = XYZ_EULER_ZYZ;
	robot_list->next->next->E_ptr->ecp_td.motion_type = ABSOLUTE;
	robot_list->next->next->E_ptr->ecp_td.motion_steps = td.internode_step_no;
	robot_list->next->next->E_ptr->ecp_td.value_in_step_no = td.value_in_step_no;
	
	strcpy( robot_list->next->next->E_ptr->ecp_td.prosody, "joy" );
*/
    
    return true;
    

}; // end: tight_coop_generator::first_step()

// ----------------------------------------------------------------------------------------------
// -----------------------------------  metoda	next_step -----------------------------------
// ----------------------------------------------------------------------------------------------

bool MP_vf_generator::next_step () {
 // Generacja trajektorii prostoliniowej o zadany przyrost polozenia i orientacji
 // Funkcja zwraca false gdy koniec generacji trajektorii
 // Funkcja zwraca true gdy generacja trajektorii bedzie kontynuowana
 // UWAGA: dzialamy na jednoelementowej liscie robotow

   int i; // licznik kolejnych wspolrzednych wektora [0..6]
 
 
 
    if ( idle_step_counter ) { // Oczekiwanie na odczyt aktualnego polozenia koncowki
     idle_step_counter--;   
     return true;
   }


 
  
	robot_m[ROBOT_IRP6_ON_TRACK]->ecp_td.instruction_type = SET;
	robot_m[ROBOT_IRP6_ON_TRACK]->ecp_td.get_type = NOTHING_DV;
	robot_m[ROBOT_IRP6_ON_TRACK]->ecp_td.get_arm_type = INVALID_END_EFFECTOR;
	
	robot_m[ROBOT_IRP6_POSTUMENT]->ecp_td.instruction_type = SET;
	robot_m[ROBOT_IRP6_POSTUMENT]->ecp_td.get_type = NOTHING_DV;
	robot_m[ROBOT_IRP6_POSTUMENT]->ecp_td.get_arm_type = INVALID_END_EFFECTOR;
	
// 	robot_list->next->next->E_ptr->ecp_td.instruction_type = SET;
	
	


	if(sensor_m[SENSOR_CAMERA_SA]->current_period==20)
	{
	
	the_first_big=0;
	
	 	 for(int i=0; i<4; i++)
			for(int j=0; j<4; j++)
				frame1[i][j]=sensor_m[SENSOR_CAMERA_SA]->image.sensor_union.camera.frame[4*i+j];

		valid_measure=(int)frame1[3][3];
			
		for(int i=0; i<4; i++)
		{
			for(int j=0; j<4; j++)
				std::cout << frame1[i][j] <<' ';
			std::cout <<std::endl;
		}

		// measure[0][0]=frame1[0][3];
		// measure[1][0]=frame1[1][3];
		// measure[2][0]=frame1[2][3];

		gammax=atan2(frame1[2][1],frame1[2][2]);
 		beta=atan2(-frame1[2][0],sqrt(frame1[0][0]*frame1[0][0]+frame1[1][0]*frame1[1][0]));
 		alfa=atan2(frame1[1][0],frame1[0][0]);
 			alfa-=0.86;
		printf("ECP: alfa=%f, beta=%f, gammax=%f\n", alfa, beta, gammax);
		printf("VALID: %d\n", valid_measure);
		
		if(valid_measure)
		measure[1][0]=alfa;
		
	/*
		if(the_first_big==0)
			measure[1][0]=alfa;
		else
			the_first_big=0;
			*/
	
// printf("1m %f s0 %f s1 %f\n", measure[1][0], stearing[1][0], stearing[1][1]);

	
			// krancowki
 		for(int i=0; i<6; i++)
 		{
 	 		if(measure[i][0]<measure_border_d[i]) 
 				measure[i][0]=measure_border_d[i];
 
 			if(measure[i][0]>measure_border_u[i]) 
 				measure[i][0]=measure_border_u[i];
 		}
/*
		
 	 		if(measure[1][0]<measure_border_d[1]) 
 			{	measure[1][0]=measure_border_d[1]; printf("D\n");}
 
 printf("1.5m %f s0 %f s1 %f\n", measure[1][0], stearing[1][0], stearing[1][1]);
 		
 
 
 			if(measure[1][0]>measure_border_u[1]) 
 			{	measure[1][0]=measure_border_u[1]; printf("U\n");}
 		
		
 		printf("2m %f s0 %f s1 %f\n", measure[1][0], stearing[1][0], stearing[1][1]);
 		
 		*/
 		
 		for (int i=0; i<6 ; i++)             
       printf("%f  ", robot_m[ROBOT_IRP6_ON_TRACK]->ecp_td.next_joint_arm_coordinates[i]);
       // printf("%f  ",robot_list->E_ptr->ecp_td.current_joint_arm_coordinates[i]);
	printf("\n");
	
	printf("PO eAAAAAAAAAAAAAAAAAAAAAAAAA\n");
 		
}		
	
	// co blokujemy
	// 1szy robot
	for (i=0; i<1; i++)
		td.coordinate_delta[i]= robot_m[ROBOT_IRP6_ON_TRACK]->ecp_td.current_joint_arm_coordinates[i];

	for (i=2; i<6; i++)
		td.coordinate_delta[i]= robot_m[ROBOT_IRP6_ON_TRACK]->ecp_td.current_joint_arm_coordinates[i];
	// 2gi robot	
// 	for (i=0; i<1; i++)
// 		coordinate_delta2[i]=robot_list->next->E_ptr->ecp_td.current_joint_arm_coordinates[i];

	for (i=1; i<6; i++)
		coordinate_delta2[i]= robot_m[ROBOT_IRP6_POSTUMENT]->ecp_td.current_joint_arm_coordinates[i];
	
	
	
 
// prawo regulacji (na lewo jest LSD) 
// stearing[1][0]=stearing[1][1] + (0.1/30)*(measure[1][0]-stearing[1][1]); // jak robot jest -0.044 na joint1 to alfa jest 0.72
 stearing[1][0]=sensor_m[SENSOR_FORCE_ON_TRACK]->image.sensor_union.force.rez[1]*0.000002+stearing[1][1] + (0.1/30)*(measure[1][0]-stearing[1][1]); // jak robot jest -0.044 na joint1 to alfa jest 0.72



 if(the_first_big==1)
printf("m %f s0 %f s1 %f\n", measure[1][0], stearing[1][0], stearing[1][1]);
 
 // stearing[3][0]=measure[3][0];
 
 // roznica w kroku -> docelowo predkosc
 for(int i=0; i<6; i++)
 	stearing_d[i][0]=stearing[i][0]-stearing[i][1];
 
 for(int i=0; i<6; i++)
 {
 	if(fabs(stearing_d[i][0])>=0)
 	{
 		if(stearing_d[i][0]>=d_u_max[i])
 			stearing[i][1]=stearing[i][1]+d_u_max[i];
 	}
	 else
	{
 		if(stearing_d[i][0]<=-1*d_u_max[i])
 			stearing[i][1]=stearing[3][1]-d_u_max[i];
 	}
 }
 
/*
if(stearing_d[i][0]>0)
	strcpy( robot_list->next->next->E_ptr->ecp_td.text, "prawo" ); 
else
	strcpy( robot_list->next->next->E_ptr->ecp_td.text, "lewo" );
*/
/*
// chuchamy na zimne - krancowki na sterowaniu 
  for(int i=0; i<6; i++)
 {
 	 if(stearing[i][0]<measure_border_d[i]) 
 		measure[i][0]=measure_border_d[i];
 
 	if(stearing[i][0]>measure_border_u[i]) 
 		measure[i][0]=measure_border_u[i];
 }
*/
if(the_first_big==1) {
	for (int i=0; i<6 ; i++)             
       printf("%f  ", robot_m[ROBOT_IRP6_ON_TRACK]->ecp_td.next_joint_arm_coordinates[i]);
       
	printf("\n");
}

td.coordinate_delta[1]=stearing[1][0];
coordinate_delta2[0]=stearing[1][0]; // robot_list->next->E_ptr->ecp_td.current_joint_arm_coordinates[0];
	
	if(the_first)
{
the_first=0;
the_second=1;
printf("XXXXXXXXXXXXXXXXFIRST\n");
for (i=0; i<6; i++)
{	
	robot_m[ROBOT_IRP6_ON_TRACK]->ecp_td.next_joint_arm_coordinates[i]= robot_m[ROBOT_IRP6_ON_TRACK]->ecp_td.current_joint_arm_coordinates[i];
	robot_m[ROBOT_IRP6_ON_TRACK]->ecp_td.next_joint_arm_coordinates[i]= robot_m[ROBOT_IRP6_ON_TRACK]->ecp_td.current_joint_arm_coordinates[i];
	measure[i][0]= robot_m[ROBOT_IRP6_ON_TRACK]->ecp_td.current_joint_arm_coordinates[i];
	measure[i][1]= robot_m[ROBOT_IRP6_ON_TRACK]->ecp_td.current_joint_arm_coordinates[i];
	measure[i][2]= robot_m[ROBOT_IRP6_ON_TRACK]->ecp_td.current_joint_arm_coordinates[i];
	
	// 2gi robot
	robot_m[ROBOT_IRP6_POSTUMENT]->ecp_td.next_joint_arm_coordinates[i]= robot_m[ROBOT_IRP6_POSTUMENT]->ecp_td.current_joint_arm_coordinates[i];
	robot_m[ROBOT_IRP6_POSTUMENT]->ecp_td.next_joint_arm_coordinates[i]= robot_m[ROBOT_IRP6_POSTUMENT]->ecp_td.current_joint_arm_coordinates[i];
	}
}
else
if(the_second)
{
printf("XXXXXXXXXXXXXXXXSEC\n");
for (i=0; i<6; i++)
{	
	robot_m[ROBOT_IRP6_ON_TRACK]->ecp_td.next_joint_arm_coordinates[i]= robot_m[ROBOT_IRP6_ON_TRACK]->ecp_td.current_joint_arm_coordinates[i];
	// 2gi robot
	robot_m[ROBOT_IRP6_POSTUMENT]->ecp_td.next_joint_arm_coordinates[i]= robot_m[ROBOT_IRP6_POSTUMENT]->ecp_td.current_joint_arm_coordinates[i];
	
	
// 	td.coordinate_delta[i]=robot_list->E_ptr->ecp_td.current_joint_arm_coordinates[i]; // DEBUG	
	pose[i][0]= robot_m[ROBOT_IRP6_ON_TRACK]->ecp_td.current_joint_arm_coordinates[i];
	pose[i][1]= robot_m[ROBOT_IRP6_ON_TRACK]->ecp_td.current_joint_arm_coordinates[i];
	pose[i][2]= robot_m[ROBOT_IRP6_ON_TRACK]->ecp_td.current_joint_arm_coordinates[i];
	measure[i][0]= robot_m[ROBOT_IRP6_ON_TRACK]->ecp_td.current_joint_arm_coordinates[i];
	measure[i][1]= robot_m[ROBOT_IRP6_ON_TRACK]->ecp_td.current_joint_arm_coordinates[i];
	measure[i][2]= robot_m[ROBOT_IRP6_ON_TRACK]->ecp_td.current_joint_arm_coordinates[i];
	stearing[i][0]= robot_m[ROBOT_IRP6_ON_TRACK]->ecp_td.current_joint_arm_coordinates[i];
	stearing[i][1]= robot_m[ROBOT_IRP6_ON_TRACK]->ecp_td.current_joint_arm_coordinates[i];
	stearing[i][3]= robot_m[ROBOT_IRP6_ON_TRACK]->ecp_td.current_joint_arm_coordinates[i];
}
	
the_second=0;
// sensor_list->E_ptr->base_period=1;
// sensor_list->E_ptr->current_period=1;

	
 	for (int i=0; i<6 ; i++)             
       printf("%f  ", robot_m[ROBOT_IRP6_ON_TRACK]->ecp_td.next_joint_arm_coordinates[i]);
       // printf("%f  ",robot_list->E_ptr->ecp_td.current_joint_arm_coordinates[i]);
	printf("\n");
	
	printf("PO eAAAAAAAAAAAAAAAAAAAAAAAAA\n");


}
else
      for (i=0; i<6; i++)
      {
     	robot_m[ROBOT_IRP6_ON_TRACK]->ecp_td.next_joint_arm_coordinates[i] = td.coordinate_delta[i];
		robot_m[ROBOT_IRP6_POSTUMENT]->ecp_td.next_joint_arm_coordinates[i] = coordinate_delta2[i]; // 2gi robot
		pose[i][2]=pose[i][1];		
		pose[i][1]=pose[i][0];
		pose[i][0]=robot_m[ROBOT_IRP6_ON_TRACK]->ecp_td.current_joint_arm_coordinates[i];
		
		stearing[i][2]=stearing[i][1];
		stearing[i][1]=stearing[i][2];
		stearing[i][1]=stearing[i][0];
		if(the_first_big==1) {
		measure[i][0]= robot_m[ROBOT_IRP6_ON_TRACK]->ecp_td.current_joint_arm_coordinates[i];
		measure[i][1]= robot_m[ROBOT_IRP6_ON_TRACK]->ecp_td.current_joint_arm_coordinates[i];
		measure[i][2]= robot_m[ROBOT_IRP6_ON_TRACK]->ecp_td.current_joint_arm_coordinates[i];
		}
		
		
	}	


 // UWAGA: dzialamy na jednoelementowej liscie robotow
   if ( robot_m[ROBOT_IRP6_ON_TRACK]->ecp_td.ecp_reply == TASK_TERMINATED ) {
   sr_ecp_msg.message("w mp task terminated");
     return false;
   }
   else {
     return true;
   }

}; // end: bool tight_coop_generator::next_step ()


} // namespace generator
} // namespace mp
} // namespace mrrocpp
