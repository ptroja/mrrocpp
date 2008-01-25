// ------------------------------------------------------------------------
// Proces:		EDP
// Plik:			edp_irp6m_effector.cc
// System:	QNX/MRROC++  v. 6.3
// Opis:		Robot IRp-6 na postumencie
//				- definicja metod klasy edp_irp6m_effector
//				- definicja funkcji return_created_efector()
//
// Autor:		tkornuta
// Data:		14.02.2007
// ------------------------------------------------------------------------

#include <stdio.h>
#include <pthread.h>

#include "common/typedefs.h"
#include "common/impconst.h"
#include "common/com_buf.h"
#include "lib/mis_fun.h"
#include "lib/mathtr.h"

// Klasa edp_irp6ot_effector.
#include "edp/irp6_mechatronika/edp_irp6m_effector.h"
// Kinematyki.
#include "edp/irp6_mechatronika/kinematic_model_irp6m_with_wrist.h"
#include "edp/irp6_mechatronika/kinematic_model_irp6m_5dof.h"

extern master_trans_t_buffer mt_tt_obj;
extern reader_buffer rb_obj;




// Konstruktor.
edp_irp6m_effector::edp_irp6m_effector () :
	edp_irp6s_effector (ROBOT_IRP6_MECHATRONIKA)
{
	
  //  Stworzenie listy dostepnych kinematyk.
  create_kinematic_models_for_given_robot();
  
	number_of_servos = IRP6_MECHATRONIKA_NUM_OF_SERVOS;
	
	reset_variables();

}; //: edp_irp6m_effector



/*--------------------------------------------------------------------------*/
void edp_irp6m_effector::arm_abs_xyz_eul_zyz_2_frame (double *p) {
                         
	double x, y, z;					// wspolrzedne wektora przesuniecia
	double alfa, beta, gamma;	// Katy Eulera
	
	// przepisanie z tablicy pakietu komunikacyjnego
	x = p[0];
	y = p[1];
	z = p[2];

	alfa = p[3];
	beta = p[4];
	gamma = p[5];
	Homog_matrix A_B_T (Homog_matrix::MTR_MECH_XYZ_EULER_ZYZ, x, y, z, alfa, beta, gamma);
	//Homog_matrix A_B_T (XYZ_EULER_ZYZ, x, y, z, alfa, beta, gamma);
	A_B_T.get_frame_tab(desired_end_effector_frame_m);

}; // end: edp_irp6s_effector::arm_abs_xyz_eul_zyz_2_frame
/*--------------------------------------------------------------------------*/


/*--------------------------------------------------------------------------*/
void edp_irp6m_effector::arm_frame_2_xyz_eul_zyz () {
 
                           // Przeksztacenie definicji koncowki z postaci
                           // FRAME do postaci XYZ_EULER_ZYZ
                           // oraz przepisanie wyniku przeksztacenia do
                           // wewntrznych struktur danych REPLY_BUFFER
	Homog_matrix A(current_end_effector_frame_m);
	switch (reply.reply_type) {
		case ARM:
		case ARM_INPUTS:
		case ARM_RMODEL:
		case ARM_RMODEL_INPUTS:
			A.get_mech_xyz_euler_zyz(reply.arm.coordinate_def.arm_coordinates);
			A.get_mech_xyz_euler_zyz(rb_obj.step_data.current_kartez_position);
			//A.get_xyz_euler_zyz(reply.arm.coordinate_def.arm_coordinates);
			//A.get_xyz_euler_zyz(rb_obj.step_data.current_kartez_position);
			break;
		default: // blad:
			throw NonFatal_error_2(STRANGE_GET_ARM_REQUEST);
	}; // end: switch (reply.reply_type)
			// dla robotow track i postument - oblicz chwytak
	if ((robot_name == ROBOT_IRP6_ON_TRACK) || (robot_name == ROBOT_IRP6_POSTUMENT))
	{
		reply.arm.coordinate_def.gripper_reg_state = servo_gripper_reg_state;
		reply.arm.coordinate_def.gripper_coordinate = current_joints[gripper_servo_nr];
	}

}; // end: edp_irp6s_effector::arm_frame_2_xyz_eul_zyz
/*--------------------------------------------------------------------------*/


/*--------------------------------------------------------------------------*/
void edp_irp6m_effector::move_arm (c_buffer *instruction) { // przemieszczenie ramienia
 // Wypenienie struktury danych transformera na podstawie parametrow polecenia
 // otrzymanego z ECP. Zlecenie transformerowi przeliczenie wspolrzednych

     
 switch ((*instruction).set_arm_type) {
    case MOTOR:
       compute_motors(instruction);
		move_servos ();
		mt_tt_obj.trans_t_to_master_order_status_ready();
    break;
    case JOINT:
       compute_joints(instruction);
    		move_servos ();
		mt_tt_obj.trans_t_to_master_order_status_ready();
   break;
    case XYZ_EULER_ZYZ:

	// zapisanie wartosci zadanej dla readera
	rb_obj.lock_mutex();	

		for (int i=0; i<6;i++)
		{	   	   
			rb_obj.step_data.current_kartez_position[i]=(*instruction).arm.coordinate_def.arm_coordinates[i];
		}
	
		rb_obj.unlock_mutex();

		compute_xyz_euler_zyz(instruction);
		move_servos ();
		mt_tt_obj.trans_t_to_master_order_status_ready();

       break;
    case XYZ_ANGLE_AXIS:
       compute_xyz_angle_axis(instruction);
    		move_servos ();
		mt_tt_obj.trans_t_to_master_order_status_ready();
       break;
    case FRAME:
       compute_frame(instruction);
    		move_servos ();
		mt_tt_obj.trans_t_to_master_order_status_ready();
       break;
    default: // blad: niezdefiniowany sposb specyfikacji pozycji koncowki
       throw NonFatal_error_2(INVALID_SET_END_EFFECTOR_TYPE);
 }; // end:   switch (instruction.set_arm_type)



// by Y - uwaga na wyjatki, po rzuceniu wyjatku nie zostanie zaktualizowany previous_set_arm_type
previous_set_arm_type = (*instruction).set_arm_type;

}; // end: edp_irp6m_effector::move_arm
/*--------------------------------------------------------------------------*/





// sprawdza stan EDP zaraz po jego uruchomieniu


void edp_irp6m_effector::servo_joints_and_frame_actualization_and_upload (void)
{
	int i;
	static int catch_nr=0;


	// wyznaczenie nowych wartosci joints and frame dla obliczen w servo
	try 
	{
		get_current_kinematic_model()->mp2i_transform(servo_current_motor_pos, servo_current_joints);

		rb_obj.lock_mutex();

		for (int j = 0; j < number_of_servos; j++)
		{	
			rb_obj.step_data.current_joints[j] = servo_current_joints[j];
		}
		
		rb_obj.unlock_mutex();



		// T.K.: Obecne wywolanie
		// get_current_kinematic_model()->i2e_transform(servo_current_joints, &servo_current_end_effector_frame, NULL);
		// zastepuje wywolaniem metody, direct_kinematics_transform(), ktora rozwiazuje proste zagadnienie kinematyki
		// bez uwzglednienia narzedzia, czyli robi to, co poprzednio i2e z TOOL = null.
		//Uwaga: w edp_conveyor_effector jest podobnie.
		get_current_kinematic_model()->direct_kinematics_transform(servo_current_joints, &servo_current_frame_wo_tool);
		Homog_matrix A(servo_current_frame_wo_tool);
		A.get_mech_xyz_euler_zyz(servo_real_kartez_pos);
		//A.get_xyz_euler_zyz(servo_real_kartez_pos);
		

		// zapisanie wartosci rzeczywistej dla readera
		rb_obj.lock_mutex();

		for (int i=0; i<6; i++) 
		{
			rb_obj.step_data.real_kartez_position[i] = servo_real_kartez_pos[i];

		}
	
		rb_obj.unlock_mutex();

		// Jesli obliczenia zwiazane z baza maja byc wykonane.
		if (get_current_kinematic_model()->global_frame_computations)
		{
			Homog_matrix tmp_eem(servo_current_frame_wo_tool);
			get_current_kinematic_model()->global_frame_transform(tmp_eem);
			tmp_eem.get_frame_tab(servo_current_frame_wo_tool);
		}//: if
		
	 
		catch_nr=0;
	}//: try
	
	catch (...)
	{
		if ((++catch_nr) == 1)
			printf("servo thread servo_joints_and_frame_actualization_and_upload throw catch exception\n");
	}//: catch
	
	pthread_mutex_lock( &edp_irp6s_effector_mutex );
	// przepisnie danych na zestaw globalny
	for (i=0; i < number_of_servos; i++)
	{
		global_current_motor_pos[i]=servo_current_motor_pos[i];
		global_current_joints[i]=servo_current_joints[i];
	};//: for

	// T.K.: Nad tym trzeba pomyslec - co w tym momencie dzieje sie z global_current_end_effector_frame?
	// Jezeli zmienna ta przechowyje polozenie bez narzedzia, to nazwa jest nie tylko nieadekwatna, a wrecz mylaca.
	copy_frame(global_current_frame_wo_tool, servo_current_frame_wo_tool);
    
	pthread_mutex_unlock( &edp_irp6s_effector_mutex );
};//: edp_irp6m_effector::servo_joints_and_frame_actualization_and_upload


/*--------------------------------------------------------------------------*/
void edp_irp6m_effector::get_arm_position (bool read_hardware, c_buffer *instruction) { // odczytanie pozycji ramienia

//   printf(" GET ARM\n");

if (read_hardware)
{
	// Uformowanie rozkazu odczytu dla SERVO_GROUP
	servo_command.instruction_code = READ;
	// Wyslanie rozkazu do SERVO_GROUP
	// Pobranie z SERVO_GROUP aktualnej pozycji silnikow
	//		printf("get_arm_position read_hardware\n");
	
	send_to_SERVO_GROUP ();
	
	// Ustawienie poprzedniej wartosci zadanej na obecnie odczytane polozenie walow silnikow
	for( int i = 0; i < number_of_servos; i++)
	{
		desired_motor_pos_new[i] = desired_motor_pos_old[i] = current_motor_pos[i];
	}
	
	  if (synchronised) 
	{
	        //  check_motor_position(desired_motor_pos_new);
	          // dla sprawdzenia ograncizen w joints i motors
	          
	    	   get_current_kinematic_model()->mp2i_transform(desired_motor_pos_new, desired_joints_tmp);
	    	   
		     for (int i=0; i< number_of_servos; i++)
		   {
		 	 	desired_joints[i] = current_joints[i] = desired_joints_tmp[i];
		   }
    	   
      }	
	
}; // end: if

 // okreslenie rodzaju wspolrzednych, ktore maja by odczytane
 // oraz adekwatne wypelnienie bufora odpowiedzi
   switch ((*instruction).get_arm_type) {
     case FRAME:
        // przeliczenie wspolrzednych do poziomu, ktory ma byc odczytany
        get_current_kinematic_model()->mp2i_transform(current_motor_pos, current_joints);
        get_current_kinematic_model()->i2e_transform(current_joints, &current_end_effector_frame_m); 
        arm_frame_2_frame();
        break;
     case   XYZ_ANGLE_AXIS:
        // przeliczenie wspolrzednych do poziomu, ktory ma byc odczytany
        get_current_kinematic_model()->mp2i_transform(current_motor_pos, current_joints);
        get_current_kinematic_model()->i2e_transform(current_joints, &current_end_effector_frame_m); 
        arm_frame_2_xyz_aa();
        break;
     case   XYZ_EULER_ZYZ:
        // przeliczenie wspolrzednych do poziomu, ktory ma byc odczytany
        get_current_kinematic_model()->mp2i_transform(current_motor_pos, current_joints);
        get_current_kinematic_model()->i2e_transform(current_joints, &current_end_effector_frame_m); 
        arm_frame_2_xyz_eul_zyz();   // dla sterowania pozycyjnego
        reply.arm_type = XYZ_EULER_ZYZ;
        break;
     case   JOINT:
        // przeliczenie wspolrzednych do poziomu, ktory ma byc odczytany
        get_current_kinematic_model()->mp2i_transform(current_motor_pos, current_joints);
        arm_joints_2_joints();
        break;
     case   MOTOR:
        arm_motors_2_motors();
        break;
     default:   // blad: nieznany sposob zapisu wspolrzednych koncowki
	printf("EFF_TYPE: %d\n",(*instruction).get_arm_type);
       throw NonFatal_error_2(INVALID_GET_END_EFFECTOR_TYPE);
   }; // end: switch (instruction.get_arm_type)

   	rb_obj.lock_mutex();// by Y
	reply.servo_step=rb_obj.step_data.step;
 	rb_obj.unlock_mutex();

}; // end: edp_irp6m_effector::get_arm_position
/*--------------------------------------------------------------------------*/





// Stworzenie modeli kinematyki dla robota IRp-6 na postumencie.
void edp_irp6m_effector::create_kinematic_models_for_given_robot(void)
{
	// Stworzenie wszystkich modeli kinematyki.
	add_kinematic_model(new kinematic_model_irp6m_with_wrist());
	add_kinematic_model(new kinematic_model_irp6m_5dof());
	// Ustawienie aktywnego modelu.
	set_kinematic_model(0);
}//: create_kinematic_models_for_given_robot


// Stworzenie obiektu edp_irp6m_effector.
edp_effector* return_created_efector ()
{
	return new edp_irp6m_effector ();
}//: return_created_efector
