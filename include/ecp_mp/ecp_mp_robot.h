#ifndef ECP_MP_ROBOT_H_
#define ECP_MP_ROBOT_H_

#include <map>
#include "lib/srlib.h"
#include "common/sensor.h"
#include "ecp_mp/transmitter.h"

struct robot_transmission_data
{

	robot_transmission_data(void) :
			instruction_type(INVALID), reply_type(ACKNOWLEDGE)
		{
		}


    INSTRUCTION_TYPE instruction_type;		// typ instrukcji
    REPLY_TYPE reply_type;					// typ odpowiedzi EDP
    edp_error error_no;						// blad wykryty w EDP

    BYTE set_type;							// typ instrukcji set: ARM/RMODEL/OUTPUTS
    BYTE get_type;							// typ instrukcji get: ARM/RMODEL/INPUTS

    // okreslenie modelu robota (narzedzia, serworegulatora, korektora kinematycznego)
    RMODEL_SPECIFICATION set_rmodel_type;	// przy jego zadawaniu
    RMODEL_SPECIFICATION get_rmodel_type;	// przy jego odczycie

    // sposob zdefiniowania polozenia koncowki
    POSE_SPECIFICATION set_arm_type;		// przy jej zadawaniu
    POSE_SPECIFICATION get_arm_type;		// przy jeg odczycie

    WORD output_values;						// zadane wartosci wyjsc binarnych
    WORD input_values;						// odczytane wartosci wejsc binarnych
    BYTE analog_input[8];					// odczytane wartosci wejsc analogowych - 8 kanalow

    MOTION_TYPE motion_type;				// sposob zadania ruchu
    INTERPOLATION_TYPE interpolation_type; // sposob interpolacji

    WORD motion_steps;						// liczba krokow ruchu zadanego (makrokroku)
    /*
     liczba krokow pierwszej fazy ruchu, czyli krok,
     w ktorym ma zostac przekazana informacja o realizacji pierwszej fazy ruchu:
     0 < value_in_step_no <= motion_steps + 1
    * dla value_in_step_no = motion_steps
      wiadomosc dotrze po zrealizowaniu makrokroku, ale informacja o polozeniu
      bedzie dotyczyc realizacji przedostatniego kroku makrokroku
    * dla value_in_step_no = motion_steps + 1
      wiadomosc dotrze po zrealizowaniu jednego kroku obiegu petli ruchu jalowego
      po zakonczeniu makrokroku, ale informacja o polozeniu bedzie dotyczyc
      realizacji calego makrokroku
    * dla value_in_step_no < motion_steps
      wiadomosc dotrze przed zrealizowaniem makrokroku i informacja o polozeniu
      bedzie dotyczyc realizacji srodkowej fazy makrokroku
    */
    WORD value_in_step_no;

    // polozenie trojscianu koncowki wzgledem ukladu bazowego
    frame_tab current_arm_frame; 			// aktualne
    frame_tab next_arm_frame;    			// wygenerowane

    // wspolrzedne XYZ + orientacja koncowki wzgledem ukladu bazowego
    double current_XYZ_ZYZ_arm_coordinates[8];	// aktualne
    double next_XYZ_ZYZ_arm_coordinates[8];		// wygenerowane

    // wspolrzedne XYZ + orientacja koncowki wzgledem ukladu bazowego
    double current_XYZ_AA_arm_coordinates[6];	// aktualne
    double next_XYZ_AA_arm_coordinates[6];		// wygenerowane

    // trojscian narzedzia wzgledem kolnierza
    frame_tab current_tool_frame;			// odczytany
    frame_tab next_tool_frame;			// wygenerowany

    // XYZ + orientacja ZYZ narzedzia wzgledem kolnierza
    double current_XYZ_ZYZ_tool_coordinates[6];		// odczytane
    double next_XYZ_ZYZ_tool_coordinates[6];		// wygenerowane

    // XYZ + orientacja AA narzedzia wzgledem kolnierza
    double current_XYZ_AA_tool_coordinates[6];		// odczytane
    double next_XYZ_AA_tool_coordinates[6];			// wygenerowane

    // wspolrzedne wewnetrzne
    double current_joint_arm_coordinates[MAX_SERVOS_NR];	// aktualne
    double next_joint_arm_coordinates[MAX_SERVOS_NR];		// wygenerowane

    // polozenia walow silnikow
    double current_motor_arm_coordinates[MAX_SERVOS_NR];	// aktualne
    double next_motor_arm_coordinates[MAX_SERVOS_NR];		// wygenerowane

    // stan w ktorym znajduje sie regulator chwytaka
    short gripper_reg_state;

    // stopien rozwarcia chwytaka
    double current_gripper_coordinate;		// odczytanu
    double next_gripper_coordinate;			// zadany

    // numer zestawu parametrow
    BYTE current_kinematic_model_no;		// odczytany
    BYTE next_kinematic_model_no;			// wygenerowany

    // numery algorytmow serworegulacji
    BYTE current_servo_algorithm_no[MAX_SERVOS_NR];		// odczytane
    BYTE next_servo_algorithm_no[MAX_SERVOS_NR];		// wygenerowane

    // numery zestawow parametrow algorytmow serworegulacji
    BYTE current_servo_parameters_no[MAX_SERVOS_NR];	// odczytane
    BYTE next_servo_parameters_no[MAX_SERVOS_NR];		// wygenerowane

    // edp speaker data
    char text[MAX_TEXT];
    char prosody[MAX_PROSODY];
    bool speaking;

    // EDP force sensor interaction
    double next_force_tool_position[3];
    double next_force_tool_weight;
    double current_force_tool_position[3];
    double current_force_tool_weight;

    // by Y - do sily

    double next_inertia[6], next_reciprocal_damping[6];
    double next_velocity[MAX_SERVOS_NR], next_force_xyz_torque_xyz[6];
    BEHAVIOUR_SPECIFICATION next_behaviour[6];
    //	bool selection_vector[6];

    // r_buffer
     // double pos_xyz_rot_xyz[6];
    double current_force_xyz_torque_xyz[6];

    // end by Y

    // moment zadany dla Dunga
    double desired_torque[MAX_SERVOS_NR];

};

class ecp_mp_robot
{
public:
    const ROBOT_ENUM robot_name; // by Y - nazwa robota (track, postument etc.)

    ecp_mp_robot(ROBOT_ENUM _robot_name);

    // destruktor by Y - do usuniecia obiektu do komunikacji z SR
    virtual ~ecp_mp_robot(void);
};

#endif
