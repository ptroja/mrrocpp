#include <math.h>
#include <assert.h>
#include <iostream.h>
#include <unistd.h>

#include "common/typedefs.h"
#include "common/impconst.h"
#include "common/com_buf.h"

#include "lib/srlib.h"
#include "ecp/common/ecp_g_tzu_cs.h"

#include "lib/mathtr.h"

tzu_simple_generator :: tzu_simple_generator(ecp_task& _ecp_task, int step) : ecp_generator (_ecp_task, true)
{ 		
	step_no = step;          	
};

static FILE *file;

/* funkcja generujaca pierwszy krok ruchu */
bool tzu_simple_generator :: first_step ( ) 
{
	printf("START first step\n");
	
	/* zmienna decydujaca o tym czy wykonalismy juz ostatni krok w zalozonym przez nas ruchu */
	finished = false; 

	/* sprawdzic co robi to odwolanie systemowe */
	ecp_t.set_ecp_reply (ECP_ACKNOWLEDGE);
	
	/* znowu jakies odwolanie systemowe tym razem cos z pobraniem komendy z mp */
	ecp_t.mp_buffer_receive_and_send ();

	/* kolejny licznik inicjalizujemy na zero, jest on inkrementowany  co krok ruchu*/	
	first_run = true;
	
	/* inicjalizacja jakis zmiennych, nie mam pojecia do czego one sluza */
	td.interpolation_node_no = 1;
	td.internode_step_no = step_no;
	td.value_in_step_no = td.internode_step_no - 2;

	/* pobieramy komende od mp i dzieki temu wiemy co mamy dalej wykonac*/
	switch ( ecp_t.mp_command_type() ) 
	{
		case NEXT_POSE: 
			cout<<"c1: "<<GET<<endl;
			cout<<"c2: "<<ARM_DV<<endl;
			cout<<"c3: "<<XYZ_EULER_ZYZ<<endl;
			cout<<"c4: "<<ABSOLUTE<<endl;
			cout<<"c5: "<<td.internode_step_no<<endl;
			cout<<"c6: "<<td.value_in_step_no<<endl;
			
			the_robot->EDP_data.instruction_type = GET;
			the_robot->EDP_data.get_type = ARM_DV; // arm - ORYGINAL
			the_robot->EDP_data.set_type = ARM_DV;

			the_robot->EDP_data.set_arm_type =  XYZ_EULER_ZYZ;			// orientacja euler'owska
			the_robot->EDP_data.get_arm_type =  XYZ_EULER_ZYZ;

			the_robot->EDP_data.motion_type = ABSOLUTE;
			the_robot->EDP_data.motion_steps = td.internode_step_no;
			the_robot->EDP_data.value_in_step_no = td.value_in_step_no;

			the_robot->create_command ();
			break;
		case STOP: cout<<"stop"<<endl;
			throw ECP_error (NON_FATAL_ERROR, ECP_STOP_ACCEPTED);
		case END_MOTION: cout<<"end motion"<<endl;
		case INVALID_COMMAND: cout<<"invalid command"<<endl;
		default:
			printf("first step in mp comm: %d\n", ecp_t.mp_command_type());
			throw ECP_error(NON_FATAL_ERROR, INVALID_MP_COMMAND);
	} // end: switch
	
	/* pierwszy krok wygenerowany */
	printf("KONIEC first step\n");
	sleep(10);
	return true;
}; // end: bool tzu_simple_generator::first_step (map <SENSOR_ENUM, sensor*>& sensor_m, robot& the_robot )
// --------------------------------------------------------------------------


// --------------------------------------------------------------------------
bool tzu_simple_generator::next_step ( ) 
{
	/* wskakuje tu jak w mp wcistne pulse ecp trigger */
	if (ecp_t.pulse_check()) 
	{
		cout<<"pulse check"<<endl;
		ecp_t.mp_buffer_receive_and_send ();
		return false;
	}
	else
	{	
		ecp_t.set_ecp_reply (ECP_ACKNOWLEDGE);
		ecp_t.mp_buffer_receive_and_send ();
	}


	// Kopiowanie danych z bufora przyslanego z EDP do
	// obrazu danych wykorzystywanych przez generator
	the_robot->get_reply();
	// Przygotowanie kroku ruchu - do kolejnego wezla interpolacji
	the_robot->EDP_data.instruction_type = SET;
	the_robot->EDP_data.set_type = ARM_DV;
	the_robot->EDP_data.get_type = NOTHING_DV;
	the_robot->EDP_data.get_arm_type = INVALID_END_EFFECTOR;
	
	// zapisanie poczatkowych wspolrzednych XYZ
	//cout<<"test: "<<ecp_t->pulse_check()<<endl;
	if(first_run)
	{
		cout<<"first_run"<<endl;
		move_status = 0;
		for(int i = 0 ; i < 6 ; i++)
		{
			current_position[i] = the_robot->EDP_data.current_XYZ_ZYZ_arm_coordinates[i];
			new_position[i] = the_robot->EDP_data.current_XYZ_ZYZ_arm_coordinates[i];
		}
		
	// zatrzymanie robota w miejscu
		for (int i = 0 ; i < 6 ; i++) 
			the_robot->EDP_data.next_XYZ_ZYZ_arm_coordinates[i] = the_robot->EDP_data.current_XYZ_ZYZ_arm_coordinates[i];
		the_robot->EDP_data.next_gripper_coordinate = the_robot->EDP_data.current_gripper_coordinate;	
		first_run = false;
	}

	// do zapisu logfile'a z ruchu	
	file = fopen("logfile.txt", "a+");
	
	switch ( move_status ) 
	{
// tu na zasadzie eksperymentow dobrac to jak ustawic katy do takiej orientacji ramienia
//		case MOVE_PHASE_ONE				// move_status == 1 - pierwsza faza ruchu - ustawienie ramienia pionowo do gory
			
//			break;
//		case MOVE_PHASE_TWO:			// move_status == 2 - druga faza ruchu - ustawienie ramienia pionowo do dolu
			
//			break;
//		case MOVE_PHASE_THREE:			// move_status == 3 - trzecia faza ruchu - jeszcze nie wiem jak w tym przypadku ustawie ramie
			
//			break;
		case MOVE_END:
			cout<<"**END**"<<endl;		
			ecp_t.set_ecp_reply (TASK_TERMINATED);
			ecp_t.mp_buffer_receive_and_send ();
			finished = true;		
			break;
//		default:
//			cout<<"!!!nierozpoznana komenda!!!"<<endl;
//			throw ECP_error(NON_FATAL_ERROR, INVALID_MP_COMMAND);
	}
	
//	sleep(0.5);
//	if(new_position[0] - current_position[0] <= 0.04 && move_status == 0)
//	{
//	new_position[0] += 0.0002;
//	new_position[1] += 0.0003;
//	new_position[2] += 0.0004;
//	new_position[3] += 0.0005;
//	new_position[4] += 0.0006;
//	new_position[5] += 0.0007;
//	the_robot->EDP_data.next_XYZ_ZYZ_arm_coordinates[0] += 0.0002;
//	the_robot->EDP_data.next_XYZ_ZYZ_arm_coordinates[1] += 0.0003;
//	the_robot->EDP_data.next_XYZ_ZYZ_arm_coordinates[2] += 0.0004;
//	the_robot->EDP_data.next_XYZ_ZYZ_arm_coordinates[3] += 0.0005;
//	the_robot->EDP_data.next_XYZ_ZYZ_arm_coordinates[4] += 0.0006;
//	the_robot->EDP_data.next_XYZ_ZYZ_arm_coordinates[5] += 0.0007;

//	}
//	else if(new_position[1] - current_position[1] <= 0.04 && move_status == 0)
//	{
//		new_position[1] += 0.0002;
//		the_robot->EDP_data.next_XYZ_ZYZ_arm_coordinates[1] += 0.0002;
//		if(new_position[1] - current_position[1]>0.04 )
//			move_status = 2;
//	}
//	else if(new_position[0] - current_position[0] >= 0.0 && move_status == 2)
//	{
//		new_position[0] -= 0.0002;
//		the_robot->EDP_data.next_XYZ_ZYZ_arm_coordinates[0] -= 0.0002;
//	}
//	else if(new_position[1] - current_position[1] >= 0.0 && move_status == 2)
//	{
//		new_position[1] -= 0.0002;
//		the_robot->EDP_data.next_XYZ_ZYZ_arm_coordinates[1] -= 0.0002;
//		if(new_position[1] - current_position[1] < 0.0)
//			move_status = -1;
//	}
	cout<<"x: "<<new_position[0]<<" , "<<"y: "<<new_position[1]<<" , "<<"z: "<<new_position[2]
	       <<"alfa: "<<new_position[3]<<"beta: "<<new_position[4]<<"gamma: "<<new_position[5]<<endl;
	//cout<<"new_position[0]: "<<new_position[0]<<" , "<<"new_position[1]: "<<new_position[1]<<" , "<<"new_position[2]: "<<new_position[2]<<endl;
	
	fprintf(file, "position: %f; %f; %f; %f; %f; %f\n", new_position[0], new_position[1], new_position[2], new_position[3], new_position[4], new_position[5]);
	fclose(file);

	switch ( ecp_t.mp_command_type() ) 
	{
		case NEXT_POSE:
			if (!finished)
				the_robot->create_command ();
			else
			{
				cout<<"end?"<<endl;
				return false;
			}
		break;
		case STOP:
			throw ECP_error (NON_FATAL_ERROR, ECP_STOP_ACCEPTED);
		case END_MOTION:
		case INVALID_COMMAND:
		default:
			printf("next step in mp comm: %d\n", ecp_t.mp_command_type());
			throw ECP_error(NON_FATAL_ERROR, INVALID_MP_COMMAND);
	}
	
	return true;
};

