// -------------------------------------------------------------------------
//                              mp_task_rk.cc
// 
// MP Master Process - methods for pouring task sporadic coordination
// 
// Ostatnia modyfikacja: styczen 2008
// -------------------------------------------------------------------------


#include <stdio.h>
#include <unistd.h>
#include <string.h>

#include "common/typedefs.h"
#include "common/impconst.h"
#include "common/com_buf.h"

#include "lib/srlib.h"
#include "mp/mp.h"
#include "mp/mp_t_pouring.h"

bool mp_task_pouring::approach(void)
{
	if(set_next_ecps_state( (int) ECP_GEN_SMOOTH, 0, "trj/pouring/irp6_ot_ap.trj", 1, ROBOT_IRP6_ON_TRACK))
	{	return true;}
	
	if(set_next_ecps_state( (int) ECP_GEN_SMOOTH, 0, "trj/pouring/irp6_p_ap.trj", 1, ROBOT_IRP6_POSTUMENT))
	{	return true;}

	if (run_ext_empty_gen_for_set_of_robots_and_wait_for_task_termin_mess_of_another_set_of_robots
	 	(2, 2, ROBOT_IRP6_ON_TRACK, ROBOT_IRP6_POSTUMENT, 
		ROBOT_IRP6_ON_TRACK, ROBOT_IRP6_POSTUMENT)) {  return true;  }
	
	return false;
}

bool mp_task_pouring::grab(void)
{
	//Ustawienie w pozycji chwytania
	if(set_next_ecps_state( (int) ECP_GEN_SMOOTH, 0, "trj/pouring/irp6_ot_grab.trj", 1, ROBOT_IRP6_ON_TRACK))
	{	return true;}
	
	if(set_next_ecps_state( (int) ECP_GEN_SMOOTH, 0, "trj/pouring/irp6_p_grab.trj", 1, ROBOT_IRP6_POSTUMENT))
	{	return true;}

	if (run_ext_empty_gen_for_set_of_robots_and_wait_for_task_termin_mess_of_another_set_of_robots
	 	(2, 2, ROBOT_IRP6_ON_TRACK, ROBOT_IRP6_POSTUMENT, 
		ROBOT_IRP6_ON_TRACK, ROBOT_IRP6_POSTUMENT)) {  return true;  }	
	//Zacisniecie szczek chwytakow
	if(set_next_ecps_state( (int) GRIP, 0, "", 1, ROBOT_IRP6_ON_TRACK))
	{	return true;}
	
	if(set_next_ecps_state( (int) GRIP, 0, "", 1, ROBOT_IRP6_POSTUMENT))
	{	return true;}

	if (run_ext_empty_gen_for_set_of_robots_and_wait_for_task_termin_mess_of_another_set_of_robots
	 	(2, 2, ROBOT_IRP6_ON_TRACK, ROBOT_IRP6_POSTUMENT, 
		ROBOT_IRP6_ON_TRACK, ROBOT_IRP6_POSTUMENT)) {  return true;  }
	
	return false;
}

bool mp_task_pouring::weight(void)
{
	if(set_next_ecps_state( (int) ECP_GEN_SMOOTH, 0, "trj/pouring/irp6_ot_weight.trj", 1, ROBOT_IRP6_ON_TRACK))
	{	return true;}
	if(set_next_ecps_state( (int) ECP_GEN_SMOOTH, 0, "trj/pouring/irp6_p_weight.trj", 1, ROBOT_IRP6_POSTUMENT))
	{	return true;}

	if (run_ext_empty_gen_for_set_of_robots_and_wait_for_task_termin_mess_of_another_set_of_robots
	 	(2, 2, ROBOT_IRP6_ON_TRACK, ROBOT_IRP6_POSTUMENT, 
		ROBOT_IRP6_ON_TRACK, ROBOT_IRP6_POSTUMENT)) {  return true;  }
	return false;
}

bool mp_task_pouring::meet(void)
{
	if(set_next_ecps_state( (int) ECP_GEN_SMOOTH, 0, "trj/pouring/irp6_ot_meet.trj", 1, ROBOT_IRP6_ON_TRACK))
	{	return true;}
	if(set_next_ecps_state( (int) ECP_GEN_SMOOTH, 0, "trj/pouring/irp6_p_meet.trj", 1, ROBOT_IRP6_POSTUMENT))
	{	return true;}

	if (run_ext_empty_gen_for_set_of_robots_and_wait_for_task_termin_mess_of_another_set_of_robots
	 	(2, 2, ROBOT_IRP6_ON_TRACK, ROBOT_IRP6_POSTUMENT, 
		ROBOT_IRP6_ON_TRACK, ROBOT_IRP6_POSTUMENT)) {  return true;  }

	if(set_next_ecps_state( (int) ECP_GEN_SMOOTH, 0, "trj/pouring/irp6_ot_track_meet.trj", 1, ROBOT_IRP6_ON_TRACK))
	{	return true;}
	if (run_ext_empty_gen (false, 1, ROBOT_IRP6_ON_TRACK)) {  return true;  }

	if(set_next_ecps_state( (int) ECP_GEN_SMOOTH, 0, "trj/pouring/irp6_ot_pour.trj", 1, ROBOT_IRP6_ON_TRACK))
	{	return true;}
	if (run_ext_empty_gen (false, 1, ROBOT_IRP6_ON_TRACK)) {  return true;  }
	
	return false;
}

bool mp_task_pouring::pour(void)
{	
 

	if(set_next_ecps_state( (int) ECP_GEN_POURING, 0, "", 1, ROBOT_IRP6_ON_TRACK))
	{	return true;}
	if (run_ext_empty_gen (false, 1, ROBOT_IRP6_ON_TRACK)) {  return true;  }

/*	if(set_next_ecps_state( (int) ECP_GEN_SMOOTH, 0, "trj/pouring/irp6_ot_pour_2.trj", 1, ROBOT_IRP6_ON_TRACK))
	{	return true;}
	if (run_ext_empty_gen (false, 1, ROBOT_IRP6_ON_TRACK)) {  return true;  }

	if(set_next_ecps_state( (int) ECP_GEN_SMOOTH, 0, "trj/pouring/irp6_ot_pour_3.trj", 1, ROBOT_IRP6_ON_TRACK))
	{	return true;}
	if (run_ext_empty_gen (false, 1, ROBOT_IRP6_ON_TRACK)) {  return true;  }
*/
/*	if(set_next_ecps_state( (int) ECP_END_POURING, 0, "", 1, ROBOT_IRP6_ON_TRACK))
	{	return true;}
	if (run_ext_empty_gen (false, 1, ROBOT_IRP6_ON_TRACK)) {  return true;  }
	*/
/*
	if(set_next_ecps_state( (int) ECP_GEN_SMOOTH, 0, "trj/pouring/irp6_ot_pour_4.trj", 1, ROBOT_IRP6_ON_TRACK))
	{	return true;}
	if (run_ext_empty_gen (false, 1, ROBOT_IRP6_ON_TRACK)) {  return true;  }
*/	
	return false;
}

bool mp_task_pouring::go_back(void)
{	
	if(set_next_ecps_state( (int) ECP_GEN_SMOOTH, 0, "trj/pouring/irp6_ot_track_put_back.trj", 1, ROBOT_IRP6_ON_TRACK))
	{	return true;}
	if (run_ext_empty_gen (false, 1, ROBOT_IRP6_ON_TRACK)) {  return true;  }
	
	if(set_next_ecps_state( (int) ECP_GEN_SMOOTH, 0, "trj/pouring/irp6_ot_go_back.trj", 1, ROBOT_IRP6_ON_TRACK))
	{	return true;}
	if(set_next_ecps_state( (int) ECP_GEN_SMOOTH, 0, "trj/pouring/irp6_p_go_back.trj", 1, ROBOT_IRP6_POSTUMENT))
	{	return true;}

	if (run_ext_empty_gen_for_set_of_robots_and_wait_for_task_termin_mess_of_another_set_of_robots
	 	(2, 2, ROBOT_IRP6_ON_TRACK, ROBOT_IRP6_POSTUMENT, 
		ROBOT_IRP6_ON_TRACK, ROBOT_IRP6_POSTUMENT)) {  return true;  }

	return false;
}

bool mp_task_pouring::put_back(void)
{	
	if(set_next_ecps_state( (int) ECP_GEN_SMOOTH, 0, "trj/pouring/irp6_ot_put_back.trj", 1, ROBOT_IRP6_ON_TRACK))
	{	return true;}
	if(set_next_ecps_state( (int) ECP_GEN_SMOOTH, 0, "trj/pouring/irp6_p_put_back.trj", 1, ROBOT_IRP6_POSTUMENT))
	{	return true;}
	
	if (run_ext_empty_gen_for_set_of_robots_and_wait_for_task_termin_mess_of_another_set_of_robots
	 	(2, 2, ROBOT_IRP6_ON_TRACK, ROBOT_IRP6_POSTUMENT, 
		ROBOT_IRP6_ON_TRACK, ROBOT_IRP6_POSTUMENT)) {  return true;  }

	//rozwarcie szczek chwytakow
	if(set_next_ecps_state( (int) LET_GO, 0, "", 1, ROBOT_IRP6_ON_TRACK))
	{	return true;}
	
	if(set_next_ecps_state( (int) LET_GO, 0, "", 1, ROBOT_IRP6_POSTUMENT))
	{	return true;}

	if (run_ext_empty_gen_for_set_of_robots_and_wait_for_task_termin_mess_of_another_set_of_robots
	 	(2, 2, ROBOT_IRP6_ON_TRACK, ROBOT_IRP6_POSTUMENT, 
		ROBOT_IRP6_ON_TRACK, ROBOT_IRP6_POSTUMENT)) {  return true;  }

	//wycofanie chwytakow
	if(set_next_ecps_state( (int) ECP_GEN_SMOOTH, 0, "trj/pouring/irp6_ot_dep_1.trj", 1, ROBOT_IRP6_ON_TRACK))
	{	return true;}
	
	if(set_next_ecps_state( (int) ECP_GEN_SMOOTH, 0, "trj/pouring/irp6_p_dep_1.trj", 1, ROBOT_IRP6_POSTUMENT))
	{	return true;}

	if (run_ext_empty_gen_for_set_of_robots_and_wait_for_task_termin_mess_of_another_set_of_robots
	 	(2, 2, ROBOT_IRP6_ON_TRACK, ROBOT_IRP6_POSTUMENT, 
		ROBOT_IRP6_ON_TRACK, ROBOT_IRP6_POSTUMENT)) {  return true;  }
}

bool mp_task_pouring::depart(void)
{
	if(set_next_ecps_state( (int) ECP_GEN_SMOOTH, 0, "trj/pouring/irp6_ot_dep_2.trj", 1, ROBOT_IRP6_ON_TRACK))
	{	return true;}
	if(set_next_ecps_state( (int) ECP_GEN_SMOOTH, 0, "trj/pouring/irp6_p_dep_2.trj", 1, ROBOT_IRP6_POSTUMENT))
	{	return true;}
	
	if (run_ext_empty_gen_for_set_of_robots_and_wait_for_task_termin_mess_of_another_set_of_robots
	 	(2, 2, ROBOT_IRP6_ON_TRACK, ROBOT_IRP6_POSTUMENT, 
		ROBOT_IRP6_ON_TRACK, ROBOT_IRP6_POSTUMENT)) {  return true;  }
	
	return false;

}

mp_task_pouring::mp_task_pouring() : mp_task()
{

};

mp_task_pouring::~mp_task_pouring()
{
 
}	

mp_task* return_created_mp_task (void)
{
	return new mp_task_pouring();
}



// methods fo mp template to redefine in concete class
void mp_task_pouring::task_initialization(void) 
{
	sr_ecp_msg->message("MP pouring loaded");
};
 

void mp_task_pouring::main_task_algorithm(void)
{
	break_state=false;

      // Oczekiwanie na zlecenie START od UI  
	sr_ecp_msg->message("MP dla zadania przelewania - wcisnij start");
	wait_for_start ();
	// Wyslanie START do wszystkich ECP 
	start_all (robot_m);

	for (;;)
	{  // Wewnetrzna petla
/*

		if (!approach())
		{
			printf("Approach skonczony\n");
		}

		if (!grab())
		{
			printf("Grab skonczony\n");
		}


		if (!weight())
		{
			printf("Weight skonczony\n");
		}
		
		if(!meet())
		{
			printf("Meet skonczony\n");
		}
*/

		if(!pour())
		{
			printf("Pour skonczony\n");
		}

/*		if (!go_back())
		{
			printf("Go back skonczony\n");
		}

		if (!put_back())
		{
			printf("Put back skonczony\n");
		}

		if (!depart())
		{
			printf("Depart skonczony\n");
		}
*/

        	// Oczekiwanie na STOP od UI
		wait_for_stop (MP_THROW);
  
    		// Wyslanie STOP do wszystkich ECP po zakonczeniu programu uzytkownika
    		terminate_all (robot_m);
    		break; 
    	} // koniec: for(;;) - wewnetrzna petla
};
