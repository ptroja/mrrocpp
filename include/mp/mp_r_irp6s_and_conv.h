#ifndef MP_R_IRP6S_AND_CONV_H_
#define MP_R_IRP6S_AND_CONV_H_

#include "mp/mp_robot.h"

class mp_irp6s_and_conv_robot : public mp_robot
{
	private:
		int servos_number;
		bool has_gripper;

	public:
		mp_irp6s_and_conv_robot (ROBOT_ENUM l_robot_name, const char* _section_name, mp_task* mp_object_l);

		// virtual void execute_motion (void); // Zlecenie wykonania ruchu przez robota
		// na poziomie MP jest to polecenie dla ECP
		// virtual void terminate_ecp (void); // Zlecenie STOP
		// virtual void start_ecp ( void );      // Zlecenie START

		virtual void create_next_pose_command (void);
		// wypelnia bufor wysylkowy do EDP na podstawie danych zawartych w swych skladowych
		// Ten bufor znajduje sie w robocie

		virtual void get_reply (void);
		virtual void get_input_reply (void);
		virtual void get_arm_reply (void);
		virtual void get_rmodel_reply (void);
		// pobiera z pakietu przeslanego z EDP informacje (aktualnie znajdujace sie
		// w klasie robot) i wstawia je do odpowiednich swoich skladowych
		// Ten bufor znajduje sie w robocie

};

#endif /*MP_R_IRP6S_AND_CONV_H_*/
