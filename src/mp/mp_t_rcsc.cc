#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <list>
#include <map>

#include "common/typedefs.h"
#include "common/impconst.h"
#include "common/com_buf.h"

#include "lib/srlib.h"
#include "mp/mp.h"
#include "mp/mp_g_force.h"
#include "mp/mp_g_vis.h"
#include "mp/mp_t_rcsc.h"
#include "ecp_mp/ecp_mp_s_schunk.h"
#include "ecp_mp/ecp_mp_s_vis.h"
#include "ecp_mp/ecp_mp_tr_rc_windows.h"


// MP_RUBIK_CUBE_SOLVER_CLASS


void mp_task_rubik_cube_solver::initiate(CUBE_COLOR up_is, CUBE_COLOR down_is, CUBE_COLOR front_is, 
		CUBE_COLOR rear_is, CUBE_COLOR left_is, CUBE_COLOR right_is)
{
	cube_state = new CubeState(up_is, down_is, front_is, rear_is, left_is, right_is);
	
	cube_initial_state = NULL;
	manipulation_sequence_computed = false;
	
};


mp_task_rubik_cube_solver::mp_task_rubik_cube_solver() : mp_task()
{

};

mp_task_rubik_cube_solver::~mp_task_rubik_cube_solver()
{
    		delete cube_state;
}	
	


bool mp_task_rubik_cube_solver::identify_colors() //DO WIZJI (przekladanie i ogladanie scian)
{

	//sekwencja poczatkowa w kolejnosci: UP, DOWN, FRONT, BACK, LEFT, RIGHT
	//cube_initial_state=BGROWY
	

	// manianka		
	cube_state->set_state(BLUE, GREEN, RED, ORANGE, WHITE, YELLOW);
	
	CUBE_TURN_ANGLE changing_order[]={CL_0, CL_0, CL_180, CL_0, CL_180, CL_0};
				
	for(int k=0; k<6; k++)
	{
		    
		if(face_turn_op(CL_0))
			return true;
		
		if (set_next_ecps_state ((int) ECP_GEN_SPEAK, 0, "oglAdam kolory na Sciance", 1, ROBOT_SPEAKER)) {  return true;  }
	
	    	   	// uruchomienie generatora empty_gen i oczekiwanie na zakonczenie obydwu generatorow ECP
	   	if (run_ext_empty_gen_for_set_of_robots_and_wait_for_task_termin_mess_of_another_set_of_robots
	 		(1, 1, ROBOT_SPEAKER, 
			ROBOT_SPEAKER)) {  return true;  }	

	    
		if (wait_ms(5000)) {  return true;  } //30 000 - OK //unrem		   //3000 - na lato na zime 5000 
		sensor_m[SENSOR_CAMERA_ON_TRACK]->initiate_reading();			
		if (wait_ms(1000)) {  return true;  }	
		sensor_m[SENSOR_CAMERA_ON_TRACK]->get_reading();
		
		for(int i=0; i<3; i++)
			for(int j=0; j<3; j++)	
				cube_state->cube_tab[k][3*i+j]=(char)sensor_m[SENSOR_CAMERA_ON_TRACK]->image.cube_face.colors[3*i+j];

				
		printf("\nFACE FACE %d:\n",k);	
		for(int i=0; i<9; i++)
		{
			switch (cube_state->cube_tab[k][i])
		 	{
				case 1: cube_state->cube_tab[k][i]='r';printf("R"); break;
				case 2: cube_state->cube_tab[k][i]='o';printf("O"); break;
				case 3: cube_state->cube_tab[k][i]='y'; printf("Y"); break;
				case 4: cube_state->cube_tab[k][i]='g'; printf("G"); break;
				case 5: cube_state->cube_tab[k][i]='b'; printf("B"); break;
				case 6: cube_state->cube_tab[k][i]='w'; printf("W"); break;
				default: cube_state->cube_tab[k][i]='o'; printf("?"); break;
		 	}
		}
		printf("\n");			
			
		if (wait_ms(1000)) {  return true;  }
		if(face_change_op(changing_order[k]))
			return true;
		    
	} //for
	return false;
}


bool mp_task_rubik_cube_solver::communicate_with_windows_solver()
{
	char c_up;
	char c_right;
	char c_front;
	char c_down;
	char c_left;
	char c_back;
//	char face_c, rot_c, curr_c;
	int s;
	int str_size;
	char cube_tab_send[54];
	char manipulation_sequence[200];

	for(int i=0; i<3; i++)
		for(int j=0; j<3; j++)	 
			cube_tab_send[2*9+3*i+j]=cube_state->cube_tab[0][3*i+j]; //rot cl 0 
			
	for(int i=0; i<3; i++)
		for(int j=0; j<3; j++)	 
			cube_tab_send[1*9+3*j+2-i]=cube_state->cube_tab[1][3*i+j]; //rot cl 90
	
	for(int i=0; i<3; i++)
		for(int j=0; j<3; j++)	 
			cube_tab_send[3*9+3*(2-j)+i]=cube_state->cube_tab[2][3*i+j]; //rot ccl 90
	
	for(int i=0; i<3; i++)
		for(int j=0; j<3; j++)	 
			cube_tab_send[5*9+3*i+j]=cube_state->cube_tab[3][3*i+j]; //rot cl 0
 
	for(int i=0; i<3; i++)
		for(int j=0; j<3; j++)	 
			cube_tab_send[4*9+3*j+2-i]=cube_state->cube_tab[4][3*i+j]; //rot cl 90
	
	for(int i=0; i<3; i++)
		for(int j=0; j<3; j++)	 
			cube_tab_send[0*9+3*j+2-i]=cube_state->cube_tab[5][3*i+j]; //rot cl 90
	
	printf("SEQ IN COLOR : %s\n",cube_tab_send);
		
	c_up=cube_tab_send[4];
	c_right=cube_tab_send[13];
	c_front=cube_tab_send[22];
	c_down=cube_tab_send[31];
	c_left=cube_tab_send[40];
	c_back=cube_tab_send[49];
			
	printf("%c %c %c %c %c %c\n", c_up, c_right, c_front, c_down, c_left, c_back);
		
	for(int i=0; i<54; i++)
	{
		  	if (cube_tab_send[i] == c_up) 
				cube_tab_send[i]='u';
			else if (cube_tab_send[i] == c_down)
				 cube_tab_send[i]='d';
			else if (cube_tab_send[i] == c_front)
				 cube_tab_send[i]='f';
			else if (cube_tab_send[i] == c_back) 
				cube_tab_send[i]='b';
			else if (cube_tab_send[i] == c_right)
				cube_tab_send[i]='r';
			else if (cube_tab_send[i] == c_left) 
				cube_tab_send[i]='l';		
	}	
			
	/*
	for(int i=0; i<54; i++)
	{
		switch (cube_tab_send[i])
		 {
		  	case 'b': cube_tab_send[i]='u'; break;
		  	case 'g': cube_tab_send[i]='d'; break;
		  	case 'o': cube_tab_send[i]='f'; break;
		  	case 'r': cube_tab_send[i]='b'; break;
		  	case 'y': cube_tab_send[i]='r'; break;
		  	case 'w': cube_tab_send[i]='l'; break;
		 }
	}		
	*/
	
	cube_tab_send[54]='\0';
			
	printf("SEQ FROM VIS : %s\n",cube_tab_send);
			
	//reszta
	// struktura pomiocnicza
	SingleManipulation single_manipulation;
	
	// czyszczenie listy
	manipulation_list.clear();
	
	for(int i=0; i<54; i++)
	{
		transmitter_m[TRANSMITTER_RC_WINDOWS]->to_va.rc_windows.rc_state[i]=cube_tab_send[i];   	
	}
	//mp_object.transmitter_m[TRANSMITTER_RC_WINDOWS]->to_va.rc_windows.rc_state[i]=patternx[i];
	transmitter_m[TRANSMITTER_RC_WINDOWS]->to_va.rc_windows.rc_state[54]='\0';
	
	if (set_next_ecps_state ((int) ECP_GEN_SPEAK, 0, "mySlE", 1, ROBOT_SPEAKER)) {  return true;  }

    	   	// uruchomienie generatora empty_gen i oczekiwanie na zakonczenie obydwu generatorow ECP
   	if (run_ext_empty_gen_for_set_of_robots_and_wait_for_task_termin_mess_of_another_set_of_robots
 		(1, 1, ROBOT_SPEAKER, 
		ROBOT_SPEAKER)) {  return true;  }	
	
	transmitter_m[TRANSMITTER_RC_WINDOWS]->t_write();
	
	
	
	transmitter_m[TRANSMITTER_RC_WINDOWS]->t_read(true);
	
	printf ("OPS: %s", transmitter_m[TRANSMITTER_RC_WINDOWS]->from_va.rc_windows.sequence);
	
	strcpy (manipulation_sequence,transmitter_m[TRANSMITTER_RC_WINDOWS]->from_va.rc_windows.sequence);
		    
	if ((manipulation_sequence[0]=='C') && (manipulation_sequence[1]=='u') && (manipulation_sequence[2]=='b') && (manipulation_sequence[3]=='e'))
	{
		printf("Jam jest daltonista. ktory Ci nie ulozy kostki\n");
		manipulation_sequence_computed = false;
		return false;
	}
	
	//sekwencja poczatkowa w kolejnosci: UP, DOWN, FRONT, BACK, LEFT, RIGHT
	//cube_initial_state=BGROWY

	
    s=0;
    str_size=0;
    for (unsigned int char_i=0; char_i < strlen(transmitter_m[TRANSMITTER_RC_WINDOWS]->from_va.rc_windows.sequence)-1; char_i ++)
    {
		if (s==0)
		{
			switch (transmitter_m[TRANSMITTER_RC_WINDOWS]->from_va.rc_windows.sequence[char_i]) 
			{
				case 'U': manipulation_sequence[str_size] = 'B'; break;
				case 'D': manipulation_sequence[str_size] = 'G'; break;
				case 'F': manipulation_sequence[str_size] = 'O'; break;
				case 'B': manipulation_sequence[str_size] = 'R'; break;
				case 'L': manipulation_sequence[str_size] = 'W'; break;
				case 'R': manipulation_sequence[str_size] = 'Y'; break;
			}
			s=1;
			str_size++;
		} 			
		else if (s==1)
		{
			switch (transmitter_m[TRANSMITTER_RC_WINDOWS]->from_va.rc_windows.sequence[char_i]) 
			{
				case ' ': manipulation_sequence[str_size] = '1'; s=0; break;
				case '2': manipulation_sequence[str_size] = '2'; s=2; break;
				case '\'': manipulation_sequence[str_size] = '3'; s=2; break;
			}
			str_size++;
		}
		else if (s==2)
		{
			s=0;
		}
			
    }
			     
	if (s==1)
	{
		str_size--;
		manipulation_sequence[str_size] = '1';
		str_size++;		    
	}
	manipulation_sequence[str_size]='\0';
    
	printf ("\n%d %d\n",str_size,strlen(manipulation_sequence));
	printf ("SEQ from win %s\n",transmitter_m[TRANSMITTER_RC_WINDOWS]->from_va.rc_windows.sequence);
	printf ("\nSEQ2 %s\n",manipulation_sequence);
		   	
	//pocztaek ukladania
	// dodawanie manipulacji do listy
	for (unsigned int char_i=0; char_i < strlen(manipulation_sequence)-1; char_i += 2)
	{
		single_manipulation.set_state(read_cube_color(manipulation_sequence[char_i]),
		read_cube_turn_angle(manipulation_sequence[char_i+1]));
		manipulation_list.push_back(single_manipulation);
	}

	if (set_next_ecps_state ((int) ECP_GEN_SPEAK, 0, "juZ ukLadam", 1, ROBOT_SPEAKER)) {  return true;  }

    	   	// uruchomienie generatora empty_gen i oczekiwanie na zakonczenie obydwu generatorow ECP
   	if (run_ext_empty_gen_for_set_of_robots_and_wait_for_task_termin_mess_of_another_set_of_robots
 		(1, 1, ROBOT_SPEAKER, 
		ROBOT_SPEAKER)) {  return true;  }	

	manipulation_sequence_computed = true;
			    
	return false;
}


bool mp_task_rubik_cube_solver::execute_manipulation_sequence()
{
	for(std::list<SingleManipulation>::iterator manipulation_list_iterator = manipulation_list.begin(); 
		 manipulation_list_iterator != manipulation_list.end(); manipulation_list_iterator++)
	{
		if (manipulate(manipulation_list_iterator->face_to_turn, manipulation_list_iterator->turn_angle))
		{
				return true;
		}
	}
	return false;
}



bool mp_task_rubik_cube_solver::manipulate(CUBE_COLOR face_to_turn, CUBE_TURN_ANGLE turn_angle )
{

	if (face_to_turn == cube_state->up)
	{
		// printf("cube_state->up\n");
		if (face_change_op(CL_90))
		{
			return true;
		}
		if (face_turn_op(turn_angle))
		{
			return true;
		}
	} else if (face_to_turn == cube_state->down)
	{
		// printf("cube_state->down\n");
		if (face_change_op(CCL_90))
		{
			return true;
		}
		if (face_turn_op(turn_angle))
		{
			return true;
		}
		
	} else if (face_to_turn == cube_state->front)
	{
		// printf("cube_state->front\n");
		if (face_change_op(CL_0))
		{
			return true;
		}
		if (face_turn_op(CL_0))
		{
			return true;
		}
		if (face_change_op(CL_90))
		{
			return true;
		}
		if (face_turn_op(turn_angle))
		{
			return true;
		}
	} else if (face_to_turn == cube_state->rear)
	{
		// printf("cube_state->rear\n");
		if (face_change_op(CL_0))
		{
			return true;
		}
		if (face_turn_op(CL_0))
		{
			return true;
		}
		if (face_change_op(CCL_90))
		{
			return true;
		}
		if (face_turn_op(turn_angle))
		{
			return true;
		}
	} else if (face_to_turn == cube_state->left)
	{
		// printf("cube_state->left\n");
		if (face_change_op(CL_0))
		{
			return true;
		}
		if (face_turn_op(turn_angle))
		{
			return true;
		}
	} else if (face_to_turn == cube_state->right)
	{
		// printf("cube_state->right\n");
		if (face_change_op(CL_180))
		{
			return true;
		}
		if (face_turn_op(turn_angle))
		{
			return true;
		}
	}
	return false;
};



// obrot sciany
bool mp_task_rubik_cube_solver::face_turn_op(CUBE_TURN_ANGLE turn_angle)
{

	// ustawienie chwytakow we wlasciwej wzajemnej orientacji


  	// wlaczenie generatora uczacego w obu robotach
	switch (turn_angle)
	{
		case CL_90:
			if (set_next_ecps_state ((int) ECP_GEN_SMOOTH, 0, "trj/rcsc/irp6ot_sm_fturn_ap_cl_90_phase_1.trj", 1, ROBOT_IRP6_ON_TRACK)) {  return true;  }
			if (set_next_ecps_state ((int) ECP_GEN_SMOOTH, 0, "trj/rcsc/irp6p_sm_fturn_ap_cl_90_phase_1.trj", 1, ROBOT_IRP6_POSTUMENT)) {  return true;  }
		break;
		case CL_0:
			if (set_next_ecps_state ((int) ECP_GEN_SMOOTH, 0, "trj/rcsc/irp6ot_sm_fturn_ap_cl_0_phase_1.trj", 1, ROBOT_IRP6_ON_TRACK)) {  return true;  }
			if (set_next_ecps_state ((int) ECP_GEN_SMOOTH, 0, "trj/rcsc/irp6p_sm_fturn_ap_cl_0_phase_1.trj", 1, ROBOT_IRP6_POSTUMENT)) {  return true;  }
		break;
		case CCL_90:
			if (set_next_ecps_state ((int) ECP_GEN_SMOOTH, 0, "trj/rcsc/irp6ot_sm_fturn_ap_ccl_90_phase_1.trj", 1, ROBOT_IRP6_ON_TRACK)) {  return true;  }
			if (set_next_ecps_state ((int) ECP_GEN_SMOOTH, 0, "trj/rcsc/irp6p_sm_fturn_ap_ccl_90_phase_1.trj", 1, ROBOT_IRP6_POSTUMENT)) {  return true;  }
		break;	
		case CL_180:
			if (set_next_ecps_state ((int) ECP_GEN_SMOOTH, 0, "trj/rcsc/irp6ot_sm_fturn_ap_cl_180_phase_1.trj", 1, ROBOT_IRP6_ON_TRACK)) {  return true;  }
			if (set_next_ecps_state ((int) ECP_GEN_SMOOTH, 0, "trj/rcsc/irp6p_sm_fturn_ap_cl_180_phase_1.trj", 1, ROBOT_IRP6_POSTUMENT)) {  return true;  }
		break;
		default:
		break;			
	}

    	// uruchomienie generatora empty_gen i oczekiwanie na zakonczenie obydwu generatorow ECP
    	if (run_ext_empty_gen_for_set_of_robots_and_wait_for_task_termin_mess_of_another_set_of_robots
	 	(3, 2, ROBOT_IRP6_ON_TRACK, ROBOT_IRP6_POSTUMENT, ROBOT_SPEAKER, 
		ROBOT_IRP6_ON_TRACK, ROBOT_IRP6_POSTUMENT)) {  return true;  }
		
	// zblizenie chwytaka tracka do niruchomego chwytaka postumenta
		
	switch (turn_angle)
	{
		case CL_90:
			if (set_next_ecps_state ((int) ECP_GEN_SMOOTH, 0, "trj/rcsc/irp6ot_sm_fturn_ap_cl_90_phase_2.trj", 1, ROBOT_IRP6_ON_TRACK)) {  return true;  }
		break;
		case CL_0:
			if (set_next_ecps_state ((int) ECP_GEN_SMOOTH, 0, "trj/rcsc/irp6ot_sm_fturn_ap_cl_0_phase_2.trj", 1, ROBOT_IRP6_ON_TRACK)) {  return true;  }
		break;
		case CCL_90:
			if (set_next_ecps_state ((int) ECP_GEN_SMOOTH, 0, "trj/rcsc/irp6ot_sm_fturn_ap_ccl_90_phase_2.trj", 1, ROBOT_IRP6_ON_TRACK)) {  return true;  }
		break;	
		case CL_180:
			if (set_next_ecps_state ((int) ECP_GEN_SMOOTH, 0, "trj/rcsc/irp6ot_sm_fturn_ap_cl_180_phase_2.trj", 1, ROBOT_IRP6_ON_TRACK)) {  return true;  }
		break;
		default:
		break;			
	}	


    	// uruchomienie generatora empty_gen i oczekiwanie na zakonczenie obydwu generatorow ECP
    	if (run_ext_empty_gen_for_set_of_robots_and_wait_for_task_termin_mess_of_another_set_of_robots
	 	(2, 1, ROBOT_IRP6_ON_TRACK, ROBOT_SPEAKER, 
		ROBOT_IRP6_ON_TRACK)) {  return true;  }
		

	// zacisniecie postumenta na kostce
	

	for (std::map <SENSOR_ENUM, sensor*>::iterator sensor_m_iterator = sensor_m.begin();
			 sensor_m_iterator != sensor_m.end(); sensor_m_iterator++)
	{
		sensor_m_iterator->second->to_vsp.parameters=1; // biasowanie czujnika
		sensor_m_iterator->second->configure_sensor();
	}



		
    	// wlaczenie generatora zacisku na kostce w robocie irp6p
    	if (set_next_ecps_state ((int) ECP_GEN_TFF_RUBIK_GRAB, (int) RCSC_RG_FROM_OPEARTOR_PHASE_1, "", 1, ROBOT_IRP6_POSTUMENT)) {  return true;  }
	// uruchomienie generatora empty_gen
	if (run_ext_empty_gen (false, 1, ROBOT_IRP6_POSTUMENT)) {  return true;  }
    	// wlaczenie generatora zacisku na kostce w robocie irp6ot
    	if (set_next_ecps_state ((int) ECP_GEN_TFF_RUBIK_GRAB, (int) RCSC_RG_FROM_OPEARTOR_PHASE_2, "", 1, ROBOT_IRP6_POSTUMENT)) {  return true;  }
	// uruchomienie generatora empty_gen
	if (run_ext_empty_gen (false, 1, ROBOT_IRP6_POSTUMENT)) {  return true;  }

	// obrot kostki


	switch (turn_angle)
	{
		case CL_90:
			if (set_next_ecps_state ((int) ECP_GEN_SPEAK, 0, "obracam kostkE", 1, ROBOT_SPEAKER)) {  return true;  }
			if (set_next_ecps_state ((int) ECP_GEN_TFF_RUBIK_FACE_ROTATE, (int) RCSC_CL_90, "", 1, ROBOT_IRP6_ON_TRACK)) {  return true;  }
			// uruchomienie generatora empty_gen
		    	if (run_ext_empty_gen_for_set_of_robots_and_wait_for_task_termin_mess_of_another_set_of_robots
			 	(2, 2, ROBOT_IRP6_ON_TRACK, ROBOT_SPEAKER, ROBOT_IRP6_ON_TRACK, ROBOT_SPEAKER)) {  return true;  }
		break;
		case CL_0:
		break;
		case CCL_90:
			if (set_next_ecps_state ((int) ECP_GEN_SPEAK, 0, "obracam kostkE", 1, ROBOT_SPEAKER)) {  return true;  }
			if (set_next_ecps_state ((int) ECP_GEN_TFF_RUBIK_FACE_ROTATE, (int) RCSC_CCL_90, "", 1, ROBOT_IRP6_ON_TRACK)) {  return true;  }
			// uruchomienie generatora empty_gen
		    	if (run_ext_empty_gen_for_set_of_robots_and_wait_for_task_termin_mess_of_another_set_of_robots
			 	(2, 2, ROBOT_IRP6_ON_TRACK, ROBOT_SPEAKER, ROBOT_IRP6_ON_TRACK, ROBOT_SPEAKER)) {  return true;  }
		break;	
		case CL_180:
			if (set_next_ecps_state ((int) ECP_GEN_SPEAK, 0, "obracam kostkE", 1, ROBOT_SPEAKER)) {  return true;  }
			if (set_next_ecps_state ((int) ECP_GEN_TFF_RUBIK_FACE_ROTATE, (int) RCSC_CL_180, "", 1, ROBOT_IRP6_ON_TRACK)) {  return true;  }	
			// uruchomienie generatora empty_gen
		    	if (run_ext_empty_gen_for_set_of_robots_and_wait_for_task_termin_mess_of_another_set_of_robots
			 	(2, 2, ROBOT_IRP6_ON_TRACK, ROBOT_SPEAKER, ROBOT_IRP6_ON_TRACK, ROBOT_SPEAKER)) {  return true;  }
		break;
		default:
		break;			
	}	

	
	// rozwarcie chwytaka tracka

    	// wlaczenie generatora zacisku na kostce w robocie irp6ot
    	if (set_next_ecps_state ((int) RCSC_GRIPPER_OPENING, (int) RCSC_GO_VAR_2, "", 1, ROBOT_IRP6_ON_TRACK)) {  return true;  }
	// uruchomienie generatora empty_gen
	if (run_ext_empty_gen (false, 1, ROBOT_IRP6_ON_TRACK)) {  return true;  }
    	// wlaczenie generatora zacisku na kostce w robocie irp6ot

	// odejscie tracka od postumenta
	
	// wlaczenie generatora uczacego  robocie irp6ot
//	if (set_next_ecps_state ((int) ECP_GEN_TEACH_IN, 0, "trj/rcsc/irp6ot_fturn_de.trj", 1, ROBOT_IRP6_ON_TRACK)) {  return true;  }
	if (set_next_ecps_state ((int) ECP_GEN_SMOOTH, 0, "trj/rcsc/irp6ot_sm_fturn_de.trj", 1, ROBOT_IRP6_ON_TRACK)) {  return true;  }
 	// uruchomienie generatora empty_gen
	if (run_ext_empty_gen (false, 1, ROBOT_IRP6_ON_TRACK)) {  return true;  }
	
	return false;
};



// zmiana sciany (przelozenie kostki)
bool mp_task_rubik_cube_solver::face_change_op(CUBE_TURN_ANGLE turn_angle)
{

	// zblizenie chwytakow

	if (set_next_ecps_state ((int) ECP_GEN_SPEAK, 0, "przekLadam kostkE", 1, ROBOT_SPEAKER)) {  return true;  }

 	   	// uruchomienie generatora empty_gen i oczekiwanie na zakonczenie obydwu generatorow ECP



  	// wlaczenie generatora uczacego w obu robotach
	switch (turn_angle)
	{
		case CL_90:
//			if (set_next_ecps_state ((int) ECP_GEN_TEACH_IN, 0, "trj/rcsc/irp6ot_fchange_de_cl_90.trj", 1, ROBOT_IRP6_ON_TRACK)) {  return true;  }
//			if (set_next_ecps_state ((int) ECP_GEN_TEACH_IN, 0, "trj/rcsc/irp6p_fchange_de_cl_90.trj", 1, ROBOT_IRP6_POSTUMENT)) {  return true;  }
			if (set_next_ecps_state ((int) ECP_GEN_SMOOTH, 0, "trj/rcsc/irp6ot_sm_fchange_ap_cl_90_phase_1.trj", 1, ROBOT_IRP6_ON_TRACK)) {  return true;  }
			if (set_next_ecps_state ((int) ECP_GEN_SMOOTH, 0, "trj/rcsc/irp6p_sm_fchange_ap_cl_90_phase_1.trj", 1, ROBOT_IRP6_POSTUMENT)) {  return true;  }
		break;
		case CL_0:
//			if (set_next_ecps_state ((int) ECP_GEN_TEACH_IN, 0, "trj/rcsc/irp6ot_fchange_de_cl_0.trj", 1, ROBOT_IRP6_ON_TRACK)) {  return true;  }
//			if (set_next_ecps_state ((int) ECP_GEN_TEACH_IN, 0, "trj/rcsc/irp6p_fchange_de_cl_0.trj", 1, ROBOT_IRP6_POSTUMENT)) {  return true;  }
			if (set_next_ecps_state ((int) ECP_GEN_SMOOTH, 0, "trj/rcsc/irp6ot_sm_fchange_ap_cl_0_phase_1.trj", 1, ROBOT_IRP6_ON_TRACK)) {  return true;  }
			if (set_next_ecps_state ((int) ECP_GEN_SMOOTH, 0, "trj/rcsc/irp6p_sm_fchange_ap_cl_0_phase_1.trj", 1, ROBOT_IRP6_POSTUMENT)) {  return true;  }
		break;
		case CCL_90:
//			if (set_next_ecps_state ((int) ECP_GEN_TEACH_IN, 0, "trj/rcsc/irp6ot_fchange_de_ccl_90.trj", 1, ROBOT_IRP6_ON_TRACK)) {  return true;  }
//			if (set_next_ecps_state ((int) ECP_GEN_TEACH_IN, 0, "trj/rcsc/irp6p_fchange_de_ccl_90.trj", 1, ROBOT_IRP6_POSTUMENT)) {  return true;  }
			if (set_next_ecps_state ((int) ECP_GEN_SMOOTH, 0, "trj/rcsc/irp6ot_sm_fchange_ap_ccl_90_phase_1.trj", 1, ROBOT_IRP6_ON_TRACK)) {  return true;  }
			if (set_next_ecps_state ((int) ECP_GEN_SMOOTH, 0, "trj/rcsc/irp6p_sm_fchange_ap_ccl_90_phase_1.trj", 1, ROBOT_IRP6_POSTUMENT)) {  return true;  }			
		break;	
		case CL_180:
//			if (set_next_ecps_state ((int) ECP_GEN_TEACH_IN, 0, "trj/rcsc/irp6ot_fchange_de_cl_180.trj", 1, ROBOT_IRP6_ON_TRACK)) {  return true;  }
//			if (set_next_ecps_state ((int) ECP_GEN_TEACH_IN, 0, "trj/rcsc/irp6p_fchange_de_cl_180.trj", 1, ROBOT_IRP6_POSTUMENT)) {  return true;  }
			if (set_next_ecps_state ((int) ECP_GEN_SMOOTH, 0, "trj/rcsc/irp6ot_sm_fchange_ap_cl_180_phase_1.trj", 1, ROBOT_IRP6_ON_TRACK)) {  return true;  }
			if (set_next_ecps_state ((int) ECP_GEN_SMOOTH, 0, "trj/rcsc/irp6p_sm_fchange_ap_cl_180_phase_1.trj", 1, ROBOT_IRP6_POSTUMENT)) {  return true;  }
		break;
		default:
		break;			
	}	

    	// uruchomienie generatora empty_gen i oczekiwanie na zakonczenie obydwu generatorow ECP
    	if (run_ext_empty_gen_for_set_of_robots_and_wait_for_task_termin_mess_of_another_set_of_robots
	 	(3, 2, ROBOT_IRP6_ON_TRACK, ROBOT_IRP6_POSTUMENT, ROBOT_SPEAKER, 
		ROBOT_IRP6_ON_TRACK, ROBOT_IRP6_POSTUMENT)) {  return true;  }
	




	switch (turn_angle)
	{
		case CL_90:
			if (set_next_ecps_state ((int) ECP_GEN_SMOOTH, 0, "trj/rcsc/irp6ot_sm_fchange_ap_cl_90_phase_2.trj", 1, ROBOT_IRP6_ON_TRACK)) {  return true;  }
		break;
		case CL_0:
			if (set_next_ecps_state ((int) ECP_GEN_SMOOTH, 0, "trj/rcsc/irp6ot_sm_fchange_ap_cl_0_phase_2.trj", 1, ROBOT_IRP6_ON_TRACK)) {  return true;  }
		break;
		case CCL_90:
			if (set_next_ecps_state ((int) ECP_GEN_SMOOTH, 0, "trj/rcsc/irp6ot_sm_fchange_ap_ccl_90_phase_2.trj", 1, ROBOT_IRP6_ON_TRACK)) {  return true;  }
		break;	
		case CL_180:
			if (set_next_ecps_state ((int) ECP_GEN_SMOOTH, 0, "trj/rcsc/irp6ot_sm_fchange_ap_cl_180_phase_2.trj", 1, ROBOT_IRP6_ON_TRACK)) {  return true;  }
		break;
		default:
		break;			
	}	


    	// uruchomienie generatora empty_gen i oczekiwanie na zakonczenie obydwu generatorow ECP
    	if (run_ext_empty_gen_for_set_of_robots_and_wait_for_task_termin_mess_of_another_set_of_robots
	 	(2, 1, ROBOT_IRP6_ON_TRACK, ROBOT_SPEAKER, 
		ROBOT_IRP6_ON_TRACK)) {  return true;  }




	// zacisniecie tracka na kostce

	for (std::map <SENSOR_ENUM, sensor*>::iterator sensor_m_iterator = sensor_m.begin();
			 sensor_m_iterator != sensor_m.end(); sensor_m_iterator++)
		{
			sensor_m_iterator->second->to_vsp.parameters=1; // biasowanie czujnika
			sensor_m_iterator->second->configure_sensor();
		}



    	// wlaczenie generatora zacisku na kostce w robocie irp6ot
    	if (set_next_ecps_state ((int) ECP_GEN_TFF_RUBIK_GRAB, (int) RCSC_RG_FCHANGE_PHASE_1, "", 1, ROBOT_IRP6_ON_TRACK)) {  return true;  }
	// uruchomienie generatora empty_gen
	if (run_ext_empty_gen (false, 1, ROBOT_IRP6_ON_TRACK)) {  return true;  }
    	// wlaczenie generatora zacisku na kostce w robocie irp6ot
    	if (set_next_ecps_state ((int) ECP_GEN_TFF_RUBIK_GRAB, (int) RCSC_RG_FCHANGE_PHASE_2, "", 1, ROBOT_IRP6_ON_TRACK)) {  return true;  }
	// uruchomienie generatora empty_gen
	if (run_ext_empty_gen (false, 1, ROBOT_IRP6_ON_TRACK)) {  return true;  }

     
    	// docisniecie chwytaka tracka do kostki
    	
    	if (set_next_ecps_state ((int) ECP_GEN_TFF_GRIPPER_APPROACH, (int) 0, "", 1, ROBOT_IRP6_ON_TRACK)) {  return true;  }
	// uruchomienie generatora empty_gen
	if (run_ext_empty_gen (false, 1, ROBOT_IRP6_ON_TRACK)) {  return true;  }
	

	// zacisniecie tracka na kostce
	
    	// wlaczenie generatora zacisku na kostce w robocie irp6ot
    	if (set_next_ecps_state ((int) ECP_GEN_TFF_RUBIK_GRAB, (int) RCSC_RG_FCHANGE_PHASE_3, "", 1, ROBOT_IRP6_ON_TRACK)) {  return true;  }
	// uruchomienie generatora empty_gen
	if (run_ext_empty_gen (false, 1, ROBOT_IRP6_ON_TRACK)) {  return true;  }
	

	
	// wstepne rozwarcie chwytaka postumenta
    	if (set_next_ecps_state ((int) RCSC_GRIPPER_OPENING, (int) RCSC_GO_VAR_1, "", 1, ROBOT_IRP6_POSTUMENT)) {  return true;  }
	// uruchomienie generatora empty_gen
	if (run_ext_empty_gen (false, 1, ROBOT_IRP6_POSTUMENT)) {  return true;  }
	

  
	 // ostateczne zacisniecie tracka na kostce    
	     
    	// wlaczenie generatora zacisku na kostce w robocie irp6ot
    	if (set_next_ecps_state ((int) ECP_GEN_TFF_RUBIK_GRAB, (int) RCSC_RG_FCHANGE_PHASE_4, "", 1, ROBOT_IRP6_ON_TRACK)) {  return true;  }
	// uruchomienie generatora empty_gen
	if (run_ext_empty_gen (false, 1, ROBOT_IRP6_ON_TRACK)) {  return true;  }
	     

	// dalsze rozwarcie chwytaka postumenta

    	if (set_next_ecps_state ((int) RCSC_GRIPPER_OPENING, (int) RCSC_GO_VAR_2, "", 1, ROBOT_IRP6_POSTUMENT)) {  return true;  }
	// uruchomienie generatora empty_gen
	if (run_ext_empty_gen (false, 1, ROBOT_IRP6_POSTUMENT)) {  return true;  }
	

	// odejscie tracka od postumenta

  	// wlaczenie generatora uczacego w obu robotach
	switch (turn_angle)
	{
		case CL_90:
//			if (set_next_ecps_state ((int) ECP_GEN_TEACH_IN, 0, "trj/rcsc/irp6ot_fchange_de_cl_90.trj", 1, ROBOT_IRP6_ON_TRACK)) {  return true;  }
//			if (set_next_ecps_state ((int) ECP_GEN_TEACH_IN, 0, "trj/rcsc/irp6p_fchange_de_cl_90.trj", 1, ROBOT_IRP6_POSTUMENT)) {  return true;  }
			if (set_next_ecps_state ((int) ECP_GEN_SMOOTH, 0, "trj/rcsc/irp6ot_sm_fchange_de_cl_90.trj", 1, ROBOT_IRP6_ON_TRACK)) {  return true;  }
//			if (set_next_ecps_state ((int) ECP_GEN_SMOOTH, 0, "trj/rcsc/irp6p_sm_fchange_de_cl_90.trj", 1, ROBOT_IRP6_POSTUMENT)) {  return true;  }
		break;
		case CL_0:
//			if (set_next_ecps_state ((int) ECP_GEN_TEACH_IN, 0, "trj/rcsc/irp6ot_fchange_de_cl_0.trj", 1, ROBOT_IRP6_ON_TRACK)) {  return true;  }
//			if (set_next_ecps_state ((int) ECP_GEN_TEACH_IN, 0, "trj/rcsc/irp6p_fchange_de_cl_0.trj", 1, ROBOT_IRP6_POSTUMENT)) {  return true;  }
			if (set_next_ecps_state ((int) ECP_GEN_SMOOTH, 0, "trj/rcsc/irp6ot_sm_fchange_de_cl_0.trj", 1, ROBOT_IRP6_ON_TRACK)) {  return true;  }
//			if (set_next_ecps_state ((int) ECP_GEN_SMOOTH, 0, "trj/rcsc/irp6p_sm_fchange_de_cl_0.trj", 1, ROBOT_IRP6_POSTUMENT)) {  return true;  }
		break;
		case CCL_90:
//			if (set_next_ecps_state ((int) ECP_GEN_TEACH_IN, 0, "trj/rcsc/irp6ot_fchange_de_ccl_90.trj", 1, ROBOT_IRP6_ON_TRACK)) {  return true;  }
//			if (set_next_ecps_state ((int) ECP_GEN_TEACH_IN, 0, "trj/rcsc/irp6p_fchange_de_ccl_90.trj", 1, ROBOT_IRP6_POSTUMENT)) {  return true;  }
			if (set_next_ecps_state ((int) ECP_GEN_SMOOTH, 0, "trj/rcsc/irp6ot_sm_fchange_de_ccl_90.trj", 1, ROBOT_IRP6_ON_TRACK)) {  return true;  }
//			if (set_next_ecps_state ((int) ECP_GEN_SMOOTH, 0, "trj/rcsc/irp6p_sm_fchange_de_ccl_90.trj", 1, ROBOT_IRP6_POSTUMENT)) {  return true;  }			
		break;	
		case CL_180:
//			if (set_next_ecps_state ((int) ECP_GEN_TEACH_IN, 0, "trj/rcsc/irp6ot_fchange_de_cl_180.trj", 1, ROBOT_IRP6_ON_TRACK)) {  return true;  }
//			if (set_next_ecps_state ((int) ECP_GEN_TEACH_IN, 0, "trj/rcsc/irp6p_fchange_de_cl_180.trj", 1, ROBOT_IRP6_POSTUMENT)) {  return true;  }
			if (set_next_ecps_state ((int) ECP_GEN_SMOOTH, 0, "trj/rcsc/irp6ot_sm_fchange_de_cl_180.trj", 1, ROBOT_IRP6_ON_TRACK)) {  return true;  }
//			if (set_next_ecps_state ((int) ECP_GEN_SMOOTH, 0, "trj/rcsc/irp6p_sm_fchange_de_cl_180.trj", 1, ROBOT_IRP6_POSTUMENT)) {  return true;  }
		break;
		default:
		break;			
	}	

    	// uruchomienie generatora empty_gen i oczekiwanie na zakonczenie obydwu generatorow ECP
    	if (run_ext_empty_gen_for_set_of_robots_and_wait_for_task_termin_mess_of_another_set_of_robots
	 	(2, 1, ROBOT_IRP6_ON_TRACK, ROBOT_SPEAKER, 
		ROBOT_IRP6_ON_TRACK)) {  return true;  }
	

	// zmiana stanu kostki

	CubeState tmp_cube_state;

	switch (turn_angle)
	{
		case CL_90:
			tmp_cube_state.set_state(cube_state->left, cube_state->right, cube_state->up, cube_state->down, 
			cube_state->front, cube_state->rear);
		break;
		case CL_0:
			tmp_cube_state.set_state(cube_state->front, cube_state->rear, cube_state->left, cube_state->right, 
			cube_state->up, cube_state->down);
		break;
		case CCL_90:
			tmp_cube_state.set_state(cube_state->right, cube_state->left, cube_state->down, cube_state->up, 
			cube_state->front, cube_state->rear);
		break;	
		case CL_180:
			tmp_cube_state.set_state(cube_state->front, cube_state->rear, cube_state->right, cube_state->left, 
			cube_state->down, cube_state->up);
		break;
		default:
		break;			
	}

	*cube_state = tmp_cube_state;

	//	cube_state->print_cube_colors();
	return false;
};


 
// dojscie
bool mp_task_rubik_cube_solver::approach_op(int mode)
{
	if (set_next_ecps_state ((int) ECP_GEN_SPEAK, 0, "jestem podatny", 1, ROBOT_SPEAKER)) {  return true;  }

    	   	// uruchomienie generatora empty_gen i oczekiwanie na zakonczenie obydwu generatorow ECP
   	if (run_ext_empty_gen_for_set_of_robots_and_wait_for_task_termin_mess_of_another_set_of_robots
 		(1, 1, ROBOT_SPEAKER, ROBOT_SPEAKER)) {  return true;  }	

	if ((config->exists("irp6p_compliant")) && ((bool) config->return_int_value("irp6p_compliant")))
	{

		// wlaczenie genrator tff_nose_run_generator w tracku
		if (set_next_ecps_state ((int) ECP_GEN_TFF_NOSE_RUN, (int) 0, "", 1, ROBOT_IRP6_POSTUMENT)) {  return true;  }
	
		// uruchomienie generatora empty_gen	
		if (run_ext_empty_gen (true, 1, ROBOT_IRP6_POSTUMENT)) {  return true;  }
	
	
	  	// przerwanie pracy generatora w ECP
		if (send_end_motion_to_ecps (1, ROBOT_IRP6_POSTUMENT)) {  return true;  }
		
	}
	else
	{
	
		// wlaczenie genrator tff_nose_run_generator w tracku
		if (set_next_ecps_state ((int) ECP_GEN_TFF_NOSE_RUN, (int) 0, "", 1, ROBOT_IRP6_ON_TRACK)) {  return true;  }
	
		// uruchomienie generatora empty_gen	
		if (run_ext_empty_gen (true, 1, ROBOT_IRP6_ON_TRACK)) {  return true;  }
	
	
	  	// przerwanie pracy generatora w ECP
		if (send_end_motion_to_ecps (1, ROBOT_IRP6_ON_TRACK)) {  return true;  }
	
	}

    	// wlaczenie generatora uczacego w obu robotach
//	if (set_next_ecps_state ((int) ECP_GEN_TEACH_IN, 0, "trj/rcsc/irp6ot_ap_1.trj", 1, ROBOT_IRP6_ON_TRACK)) {  return true;  }
	if (set_next_ecps_state ((int) ECP_GEN_SMOOTH, 0, "trj/rcsc/irp6ot_sm_ap_1.trj", 1, ROBOT_IRP6_ON_TRACK)) {  return true;  }
//	if (set_next_ecps_state ((int) ECP_GEN_TEACH_IN, 0, "trj/rcsc/irp6p_ap_1.trj", 1, ROBOT_IRP6_POSTUMENT)) {  return true;  }
	if (set_next_ecps_state ((int) ECP_GEN_SMOOTH, 0, "trj/rcsc/irp6p_sm_ap_1.trj", 1, ROBOT_IRP6_POSTUMENT)) {  return true;  }

    	// uruchomienie generatora empty_gen i oczekiwanie na zakonczenie obydwu generatorow ECP

//	(1, 1, ROBOT_IRP6_ON_TRACK, ROBOT_IRP6_POSTUMENT)) {  return true;  }	
     
    	if (run_ext_empty_gen_for_set_of_robots_and_wait_for_task_termin_mess_of_another_set_of_robots
 		(3, 2, ROBOT_IRP6_ON_TRACK, ROBOT_IRP6_POSTUMENT, ROBOT_SPEAKER, 
		ROBOT_IRP6_ON_TRACK, ROBOT_IRP6_POSTUMENT)) {  return true;  }
		
		
	if (set_next_ecps_state ((int) ECP_GEN_SPEAK, 0, "jestem robotem usLugowym", 1, ROBOT_SPEAKER)) {  return true;  }     
   	// wlaczenie generatora uczacego  robocie irp6ot
   // 	if (set_next_ecps_state ((int) ECP_GEN_TEACH_IN, 0, "trj/rcsc/irp6ot_ap_2.trj", 1, ROBOT_IRP6_ON_TRACK)) {  return true;  }
    	if (set_next_ecps_state ((int) ECP_GEN_SMOOTH, 0, "trj/rcsc/irp6ot_sm_ap_2.trj", 1, ROBOT_IRP6_ON_TRACK)) {  return true;  }
/*
   	if (run_ext_empty_gen_for_set_of_robots_and_wait_for_task_termin_mess_of_another_set_of_robots
 		(3, 1, ROBOT_IRP6_ON_TRACK, ROBOT_IRP6_POSTUMENT, ROBOT_SPEAKER, 
		ROBOT_SPEAKER)) {  return true;  }
*/

   	if (run_ext_empty_gen_for_set_of_robots_and_wait_for_task_termin_mess_of_another_set_of_robots
 		(3, 2, ROBOT_IRP6_ON_TRACK, ROBOT_IRP6_POSTUMENT, ROBOT_SPEAKER, 
		ROBOT_SPEAKER, ROBOT_IRP6_ON_TRACK)) {  return true;  }
		
//	if (set_next_ecps_state ((int) ECP_GEN_TEACH_IN, 0, "trj/rcsc/irp6ot_ap_2.trj", 1, ROBOT_IRP6_ON_TRACK)) {  return true;  }	
	if (set_next_ecps_state ((int) ECP_GEN_SPEAK, 0, "uLoZe kostkE rubika", 1, ROBOT_SPEAKER)) {  return true;  }     
	
   	if (run_ext_empty_gen_for_set_of_robots_and_wait_for_task_termin_mess_of_another_set_of_robots
 		(3, 1, ROBOT_IRP6_ON_TRACK, ROBOT_IRP6_POSTUMENT, ROBOT_SPEAKER, 
		ROBOT_SPEAKER)) {  return true;  }
     
	// powiedzenie 
	if (set_next_ecps_state ((int) ECP_GEN_SPEAK, 0, "podaj kostkE", 1, ROBOT_SPEAKER)) {  return true;  }


	// uruchomienie generatora empty_gen
	if (run_ext_empty_gen_for_set_of_robots_and_wait_for_task_termin_mess_of_another_set_of_robots
 		(2, 2, ROBOT_IRP6_ON_TRACK, ROBOT_SPEAKER, 
		ROBOT_IRP6_ON_TRACK, ROBOT_SPEAKER)) {  return true;  }


    	// wlaczenie generatora transparentnego w obu robotach
    	if (set_next_ecps_state ((int) ECP_GEN_TRANSPARENT, (int) 0, "", 1, ROBOT_IRP6_ON_TRACK)) {  return true;  }
    	if (set_next_ecps_state ((int) ECP_GEN_TRANSPARENT, (int) 0, "", 1, ROBOT_IRP6_POSTUMENT)) {  return true;  }
     
	
	// opcjonalne serwo wizyjne
	if (mode)
	{
 		mp_seven_eye_generator eyegen(*this, 4);
 		eyegen.robot_m[ROBOT_IRP6_ON_TRACK] = robot_m[ROBOT_IRP6_ON_TRACK]; 
	    	eyegen.sensor_m[SENSOR_CAMERA_SA] = sensor_m[SENSOR_CAMERA_SA];
	
		if (Move ( eyegen) )
		{
		     return true;
	     }
	}
	
	if (send_end_motion_to_ecps (2, ROBOT_IRP6_ON_TRACK, ROBOT_IRP6_POSTUMENT)) {  return true;  }

	if (wait_ms(500)) {  return true;  }	

	// zacisniecie chwytaka na kostce

    	// wlaczenie generatora zacisku na kostce w robocie irp6ot
    	if (set_next_ecps_state ((int) ECP_GEN_TFF_RUBIK_GRAB, (int) RCSC_RG_FROM_OPEARTOR_PHASE_1, "", 1, ROBOT_IRP6_ON_TRACK)) {  return true;  }
	// uruchomienie generatora empty_gen
	if (run_ext_empty_gen_for_set_of_robots_and_wait_for_task_termin_mess_of_another_set_of_robots
 		(2, 1, ROBOT_IRP6_ON_TRACK, ROBOT_SPEAKER, 
		ROBOT_IRP6_ON_TRACK)) {  return true;  }
    	// wlaczenie generatora zacisku na kostce w robocie irp6ot
    	if (set_next_ecps_state ((int) ECP_GEN_TFF_RUBIK_GRAB, (int) RCSC_RG_FROM_OPEARTOR_PHASE_2, "", 1, ROBOT_IRP6_ON_TRACK)) {  return true;  }
	// uruchomienie generatora empty_gen
   	if (run_ext_empty_gen_for_set_of_robots_and_wait_for_task_termin_mess_of_another_set_of_robots
 		(2, 2, ROBOT_IRP6_ON_TRACK, ROBOT_SPEAKER, 
		ROBOT_IRP6_ON_TRACK, ROBOT_SPEAKER)) {  return true;  }

	if (set_next_ecps_state ((int) ECP_GEN_SPEAK, 0, "puSC kostkE", 1, ROBOT_SPEAKER)) {  return true;  }

    	   	// uruchomienie generatora empty_gen i oczekiwanie na zakonczenie obydwu generatorow ECP
   	if (run_ext_empty_gen_for_set_of_robots_and_wait_for_task_termin_mess_of_another_set_of_robots
 		(1, 1, ROBOT_SPEAKER, ROBOT_SPEAKER)) {  return true;  }	

	if (wait_ms(1000)) {  return true;  }

 	// wlaczenie generatora uczacego w obu robotach
//	if (set_next_ecps_state ((int) ECP_GEN_TEACH_IN, 0, "trj/rcsc/irp6ot_ap_3.trj", 1, ROBOT_IRP6_ON_TRACK)) {  return true;  }
	if (set_next_ecps_state ((int) ECP_GEN_SMOOTH, 0, "trj/rcsc/irp6ot_sm_ap_3.trj", 1, ROBOT_IRP6_ON_TRACK)) {  return true;  }

    	// uruchomienie generatora empty_gen i oczekiwanie na zakonczenie obydwu generatorow ECP

//	(1, 1, ROBOT_IRP6_ON_TRACK, ROBOT_IRP6_POSTUMENT)) {  return true;  }	
     
    	if (run_ext_empty_gen_for_set_of_robots_and_wait_for_task_termin_mess_of_another_set_of_robots
 		(3, 2, ROBOT_IRP6_ON_TRACK, ROBOT_IRP6_POSTUMENT, ROBOT_SPEAKER, 
		ROBOT_IRP6_ON_TRACK, ROBOT_IRP6_POSTUMENT)) {  return true;  }


	return false;
};


// odejscie
bool mp_task_rubik_cube_solver::departure_op()
{

	if (set_next_ecps_state ((int) ECP_GEN_SPEAK, 0, "skoNczyLem", 1, ROBOT_SPEAKER)) {  return true;  }

    	// wlaczenie generatora uczacego w obu robotach
//	if (set_next_ecps_state ((int) ECP_GEN_TEACH_IN, 0, "trj/rcsc/irp6ot_de_1.trj", 1, ROBOT_IRP6_ON_TRACK)) {  return true;  }
	if (set_next_ecps_state ((int) ECP_GEN_SMOOTH, 0, "trj/rcsc/irp6ot_sm_de_1.trj", 1, ROBOT_IRP6_ON_TRACK)) {  return true;  }
//	if (set_next_ecps_state ((int) ECP_GEN_TEACH_IN, 0, "trj/rcsc/irp6p_de_1.trj", 1, ROBOT_IRP6_POSTUMENT)) {  return true;  }
	if (set_next_ecps_state ((int) ECP_GEN_SMOOTH, 0, "trj/rcsc/irp6p_sm_de_1.trj", 1, ROBOT_IRP6_POSTUMENT)) {  return true;  }


    	if (run_ext_empty_gen_for_set_of_robots_and_wait_for_task_termin_mess_of_another_set_of_robots
	 	(3, 1, ROBOT_IRP6_ON_TRACK, ROBOT_IRP6_POSTUMENT, ROBOT_SPEAKER, 
		ROBOT_SPEAKER)) {  return true;  }

	if (set_next_ecps_state ((int) ECP_GEN_SPEAK, 0, "jadE pracowaC do angli.", 1, ROBOT_SPEAKER)) {  return true;  }
	//	if (set_next_ecps_state ((int) ECP_GEN_SPEAK, 0, "jak sie paNstwu podobaLo", 1, ROBOT_SPEAKER)) {  return true;  }

    	// uruchomienie generatora empty_gen i oczekiwanie na zakonczenie obydwu generatorow ECP
    	if (run_ext_empty_gen_for_set_of_robots_and_wait_for_task_termin_mess_of_another_set_of_robots
	 	(3, 2, ROBOT_IRP6_ON_TRACK, ROBOT_IRP6_POSTUMENT, ROBOT_SPEAKER, 
		ROBOT_IRP6_ON_TRACK, ROBOT_IRP6_POSTUMENT)) {  return true;  }

	return false;
};


bool mp_task_rubik_cube_solver::gripper_opening(double track_increment, double postument_increment, int motion_time)
{

	trajectory_description tdes;

	tdes.arm_type = XYZ_EULER_ZYZ;
	tdes.interpolation_node_no = 1;
	tdes.internode_step_no = motion_time;
	tdes.value_in_step_no = tdes.internode_step_no - 2;
	// Wspolrzedne kartezjanskie XYZ i katy Eulera ZYZ
	tdes.coordinate_delta[0] = 0.0; // przyrost wspolrzednej X
	tdes.coordinate_delta[1] = 0.0;// przyrost wspolrzednej Y
	tdes.coordinate_delta[2] = 0.0;   // przyrost wspolrzednej Z
	tdes.coordinate_delta[3] = 0.0;   // przyrost wspolrzednej FI
	tdes.coordinate_delta[4] = 0.0;   // przyrost wspolrzednej TETA
	tdes.coordinate_delta[5] = 0.0;   // przyrost wspolrzednej PSI
	//	tdes.coordinate_delta[6] = 0.0;   // przyrost wspolrzednej PSI		
	tdes.coordinate_delta[6] = track_increment;   // przyrost wspolrzednej PSI		
	
	trajectory_description tdes2;

	tdes2.arm_type = XYZ_EULER_ZYZ;
	tdes2.interpolation_node_no = 1;
	tdes2.internode_step_no = motion_time;
	tdes2.value_in_step_no = tdes.internode_step_no - 2;
	// Wspolrzedne kartezjanskie XYZ i katy Eulera ZYZ
	tdes2.coordinate_delta[0] = 0.0; // przyrost wspolrzednej X
	tdes2.coordinate_delta[1] = 0.0;// przyrost wspolrzednej Y
	tdes2.coordinate_delta[2] = 0.0;   // przyrost wspolrzednej Z
	tdes2.coordinate_delta[3] = 0.0;   // przyrost wspolrzednej FI
	tdes2.coordinate_delta[4] = 0.0;   // przyrost wspolrzednej TETA
	tdes2.coordinate_delta[5] = 0.0;   // przyrost wspolrzednej PSI
	//	tdes2.coordinate_delta[6] = 0.02;   // przyrost wspolrzednej PSI
	tdes2.coordinate_delta[6] = postument_increment;   // przyrost wspolrzednej PSI
		
	// Generator trajektorii prostoliniowej
	mp_tight_coop_generator tcg(*this, tdes, tdes2);
	tcg.robot_m = robot_m;

	if (Move ( tcg))
	{
	    return true;
     }

	return false;
}



mp_task* return_created_mp_task (void)
{
	return new mp_task_rubik_cube_solver();
}



// methods fo mp template to redefine in concete class
void mp_task_rubik_cube_solver::task_initialization(void) 
{
	// Powolanie czujnikow
	sensor_m[SENSOR_FORCE_ON_TRACK] = 
		new ecp_mp_schunk_sensor (SENSOR_FORCE_ON_TRACK, "[vsp_force_irp6ot]", *this);
		
	sensor_m[SENSOR_FORCE_POSTUMENT] = 
		new ecp_mp_schunk_sensor (SENSOR_FORCE_POSTUMENT, "[vsp_force_irp6p]", *this);
		
	sensor_m[SENSOR_CAMERA_ON_TRACK] = 
		new ecp_mp_vis_sensor (SENSOR_CAMERA_ON_TRACK, "[vsp_vis_eih]", *this);
		
	sensor_m[SENSOR_CAMERA_SA] = 
		new ecp_mp_vis_sensor (SENSOR_CAMERA_SA, "[vsp_vis_sac]", *this);
	
	// Konfiguracja wszystkich czujnikow	
	for (std::map <SENSOR_ENUM, sensor*>::iterator sensor_m_iterator = sensor_m.begin();
		 sensor_m_iterator != sensor_m.end(); sensor_m_iterator++)
	{
		sensor_m_iterator->second->to_vsp.parameters=1; // biasowanie czujnika
		sensor_m_iterator->second->configure_sensor();
	}
			
	usleep(1000*100);
	
	// dodanie transmitter'a
	transmitter_m[TRANSMITTER_RC_WINDOWS] = 
		new rc_windows_transmitter (TRANSMITTER_RC_WINDOWS, "[transmitter_rc_windows]", *this);
		
	sr_ecp_msg->message("MP rcsc loaded");
};
 

void mp_task_rubik_cube_solver::main_task_algorithm(void)
{

	break_state = false;
  
    // odczyt konfiguracji manipulacji
    if (cube_initial_state) delete[] cube_initial_state;
	cube_initial_state = config->return_string_value("cube_initial_state");
	//	enum CUBE_COLOR {UKNOWN, RED, YELLOW, GREEN, BLUE, ORANGE, WHITE};
	//	 cube_state::set_state(CUBE_COLOR up_is, CUBE_COLOR down_is, CUBE_COLOR front_is, 
	//		CUBE_COLOR rear_is, CUBE_COLOR left_is, CUBE_COLOR right_is)

	initiate (read_cube_color(cube_initial_state[0]), 
		read_cube_color(cube_initial_state[1]), read_cube_color(cube_initial_state[2]),  read_cube_color(cube_initial_state[3]), 
		read_cube_color(cube_initial_state[4]), read_cube_color(cube_initial_state[5]));

	 // printf("przed wait for start \n");
      // Oczekiwanie na zlecenie START od UI  
	sr_ecp_msg->message("MP dla kostki Rubika - wcisnij start");
	wait_for_start ();
	// Wyslanie START do wszystkich ECP 
	start_all (robot_m);

	for (;;)
	{  // Wewnetrzna petla

       	// Zlecenie wykonania kolejnego makrokroku
		// printf("po start all \n");
		 for(;;)
		{
			sr_ecp_msg->message("Nowa seria");
			for (std::map <SENSOR_ENUM, sensor*>::iterator sensor_m_iterator = sensor_m.begin();
				 sensor_m_iterator != sensor_m.end(); sensor_m_iterator++)
			{
				sensor_m_iterator->second->to_vsp.parameters=1; // biasowanie czujnika
				sensor_m_iterator->second->configure_sensor();
			}

			// przechwycenie kostki
			if (approach_op( config->return_int_value("vis_servoing"))){

		          break;
		    }

			// IDENTIFY COLORS
			if (identify_colors()){

		          break;
		    }
		
		 	if (communicate_with_windows_solver())
			{

		          break;
		    }

			if (manipulation_sequence_computed)
			{

			    // wykonanie sekwencji manipulacji
			     if (face_turn_op(CL_0)){

			          break;
			    }
			    
				    
			    	if (execute_manipulation_sequence()){

			          break;
			    }
			    
			     // zakonczenie zadania
			    
				if (face_change_op(CL_0)){

			          break;
			    }
			}

		    	if (departure_op()){		    	
		          break;
		    }

	        wait_for_stop (MP_THROW);// by Y - wlaczony tryb
	        // Wyslanie STOP do wszystkich ECP po zakonczeniu programu uzytkownika
	        terminate_all (robot_m);
	        
	         break;
		        
      	}

		break;			

      } // koniec: for(;;) - zewnetrzna petla
};
