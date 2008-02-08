// -------------------------------------------------------------------------
//                            mp.h
// Definicje struktur danych i metod dla procesow MP
//
// Ostatnia modyfikacja: 16.04.98
// -------------------------------------------------------------------------

#if !defined(__MP_H)
#define __MP_H

#include <sys/iofunc.h>
#include <sys/dispatch.h>

#include "ecp_mp/ecp_mp_task.h"
#include "lib/configurator.h"
#include "lib/mp_timer.h"

class mp_generator;
class mp_robot;

enum WAIT_FOR_STOP_ENUM {
	MP_EXIT,
	MP_THROW
};

// struktura zwracana przez funkcje mp_receive_ecp_pulse
typedef struct mp_receive_ecp_pulse_return {
	uint32_t nd; // deskryptor wezla na ktorym jest powolane ECP
	pid_t ECP_pid;
	bool rt;
	int32_t scoid; // server connection id
	char pulse_code;
	int rcvid;
	uint64_t e; // errno
} mp_receive_ecp_pulse_return_t;

// struktura wykorzystywana przez funkcje mp_receive_pulse
typedef struct mp_receive_pulse_struct {
	_msg_info msg_info;
	_pulse_msg pulse_msg;
	int rcvid;
	uint64_t e;       // Kod bledu systemowego
} mp_receive_pulse_struct_t;

// ---------------------------------------------------------------
class MP_main_error {  // Klasa obslugi bledow poziomie MP
  public:
    const uint64_t error_class;
    const uint64_t mp_error;
    MP_main_error (uint64_t err0, uint64_t err1)
	 : error_class(err0), mp_error(err1) { };
}; // end: class MP_main_error
// ---------------------------------------------------------------

// na podstawie ecp_taught_in_pose
// ------------------------------------------------------------------------
class mp_taught_in_pose {
public:
  POSE_SPECIFICATION arm_type;
  double motion_time;
  double coordinates[MAX_SERVOS_NR];
  double irp6p_coordinates[MAX_SERVOS_NR];

  int extra_info; // by Y uzupelnienie struktury o dodatkowe pole, do dowolnego wykorzystania

  mp_taught_in_pose (void);
  mp_taught_in_pose (POSE_SPECIFICATION at, double mt, double* c);

  mp_taught_in_pose (POSE_SPECIFICATION at, double mt, double* c, double* irp6p_c);

  mp_taught_in_pose (POSE_SPECIFICATION at, double mt, int e_info, double* c);
}; // end:class mp_taught_in_pose
// ------------------------------------------------------------------------

// ------------------------------------------------------------------------
struct robot_ECP_transmission_data {
  MP_COMMAND mp_command;                // polecenie przesylane z MP do ECP
  ECP_REPLY  ecp_reply;                 // odpowiedz z ECP do MP


	int mp_2_ecp_next_state; // skojarzone z NEXT_STATE
	int mp_2_ecp_next_state_variant; // skojarzone z NEXT_STATE
	char mp_2_ecp_next_state_string[MP_2_ECP_STRING_SIZE]; // skojarzone z NEXT_STATE


  char text[MAX_TEXT]; // MAC7
  char prosody[MAX_PROSODY]; // MAC7
  bool speaking; // MAC7

  INSTRUCTION_TYPE instruction_type;    // typ instrukcji: SET, GET, SET_GET, SYNCHRO, QUERY
  BYTE set_type;                        // typ instrukcji set: ARM/RMODEL/OUTPUTS
  BYTE get_type;                        // typ instrukcji get: ARM/RMODEL/INPUTS
  RMODEL_SPECIFICATION set_rmodel_type; // sposob zdefiniowania narzedzia przy jego zadawaniu:
                                        // TOOL_FRAME / TOOL_XYZ_EULER_ZYZ / TOOL_XYZ_ANGLE_AXIS /  TOOL_AS_XYZ_EULER_ZY /
                                        // ARM_KINEMATIC_MODEL / SERVO_ALGORITHM
  RMODEL_SPECIFICATION get_rmodel_type; // sposob zdefiniowania narzedzia przy jego odczycie:
                                        // TOOL_FRAME / TOOL_XYZ_EULER_ZYZ / TOOL_XYZ_ANGLE_AXIS / TOOL_AS_XYZ_EULER_ZY /
                                        // ARM_KINEMATIC_MODEL / SERVO_ALGORITHM
  POSE_SPECIFICATION set_arm_type;      // sposob zdefiniowania polozenia zadanego
                                        // koncowki: MOTOR / JOINT /
                                        // FRAME / XYZ_EULER_ZYZ / XYZ_ANGLE_AXIS
  POSE_SPECIFICATION get_arm_type;      // sposob zdefiniowania polozenia odcztanego
                                        // koncowki: MOTOR / JOINT /
                                        // FRAME / XYZ_EULER_ZYZ / XYZ_ANGLE_AXIS
  WORD output_values;                   // zadane wartosci wyjsc binarnych
  WORD input_values;                    // odczytane wartosci wejsc binarnych
  BYTE analog_input[8];            // odczytane wartosci wejsc analogowych - 8 kanalow
  MOTION_TYPE motion_type;       // sposob zadania ruchu: ABSOLUTE/RELATIVE
  WORD motion_steps;             // liczba krokow ruchu zadanego (makrokroku)
  WORD value_in_step_no;         // liczba krokow pierwszej fazy ruchu, czyli
                                 // krok, w ktorym ma zostac przekazana
                                 // informacja o realizacji pierwszej fazy
                                 // ruchu:
                                 // 0 < value_in_step_no <= motion_steps + 1
                                 // Dla value_in_step_no = motion_steps
                                 // wiadomosc dotrze po zrealizowaniu makrokroku,
                                 // ale informacja o polozeniu bedzie dotyczyc
                                 // realizacji przedostatniego kroku makrokroku.
                                 // Dla value_in_step_no = motion_steps + 1
                                 // wiadomosc dotrze po zrealizowaniu jednego
                                 // kroku obiegu petli ruchu jalowego po
                                 // zakonczeniu makrokroku,
                                 // ale informacja o polozeniu bedzie dotyczyc
                                 // realizacji calego makrokroku.
                                 // Dla value_in_step_no < motion_steps
                                 // wiadomosc dotrze przed zrealizowaniem makrokroku
                                 // i informacja o polozeniu bedzie dotyczyc
                                 // realizacji srodkowej fazy makrokroku.
  frame_tab current_arm_frame_m;   // aktualne polozenie trojscianu koncowki
                                 // wzgledem ukladu bazowego
  frame_tab next_arm_frame_m;      // wygenerowane polozenie trojscianu koncowki
                                 // wzgledem ukladu bazowego
  double current_XYZ_ZYZ_arm_coordinates[6];  // aktualne wspolrzedne XYZ +
                                 // orientacja koncowki wzgledem ukladu bazowego
  double next_XYZ_ZYZ_arm_coordinates[6];  // wygenerowane wspolrzedne XYZ +
                                 // orientacja koncowki wzgledem ukladu bazowego

// by Y  do sily

	short gripper_reg_state; // stan w ktorym znajduje sie regulator chwytaka
	double current_gripper_coordinate; // odczytanu stopien rozwarcia chwytaka
	double next_gripper_coordinate; // zadany stopien rozwarcia chwytaka

  double current_XYZ_AA_arm_coordinates[6];  // aktualne wspolrzedne XYZ +
                                 // orientacja koncowki wzgledem ukladu bazowego
  double next_XYZ_AA_arm_coordinates[6];   // wygenerowane  wspolrzedne XYZ +
                                 // orientacja koncowki wzgledem ukladu bazowego
  double current_joint_arm_coordinates[MAX_SERVOS_NR];
                                 // aktualne wspolrzedne wewnetrzne
  double next_joint_arm_coordinates[MAX_SERVOS_NR];
                                 // wygenerowane wspolrzedne wewnetrzne
  double current_motor_arm_coordinates[MAX_SERVOS_NR];
                                 // aktualne polozenia walow silnikow
  double next_motor_arm_coordinates[MAX_SERVOS_NR];
                                 // wygenerowane polozenia walow silnikow
  frame_tab current_tool_frame_m;  // odczytany trojscian narzedzia wzgledem kolnierza
  frame_tab next_tool_frame_m;     // wygenerowany trojscian narzedzia wzgledem kolnierza
  double current_XYZ_ZYZ_tool_coordinates[6];         // odczytane XYZ + orientacja ZYZ
                                                      // narzedzia wzgledem kolnierza
  double next_XYZ_ZYZ_tool_coordinates[6];            // wygenerowane XYZ + orientacja ZYZ
                                                      // narzedzia wzgledem kolnierza
  double current_XYZ_AA_tool_coordinates[6];          // odczytane XYZ + orientacja AA
                                                      // narzedzia wzgledem kolnierza
  double next_XYZ_AA_tool_coordinates[6];             // wygenerowane XYZ + orientacja AA
                                                      // narzedzia wzgledem kolnierza

	// dla POSE_FORCE_TORQUE_AT_FRAME
	// c_buffer

	double MPtoECP_inertia[6], MPtoECP_reciprocal_damping[6];
	double MPtoECP_position_velocity[MAX_SERVOS_NR], MPtoECP_force_xyz_torque_xyz[6];
	BEHAVIOUR_SPECIFICATION MPtoECP_behaviour[6];
//	bool MPselection_vector[6];


	// r_buffer
	frame_tab  MPcurrent_beggining_arm_frame_m;      // trojscian koncowki wzgledem ukladu bazowego
	frame_tab  MPcurrent_predicted_arm_frame_m;      // trojscian koncowki wzgledem ukladu bazowego
	frame_tab  MPcurrent_present_arm_frame_m;      // trojscian koncowki wzgledem ukladu bazowego
	double ECPtoMP_force_xyz_torque_xyz[6];



  BYTE current_kinematic_model_no;                    // odczytany numer zestawu parametrow
                                                      // modelu kinematyki
  BYTE next_kinematic_model_no;                       // wygenerowany numer zestawu
                                                      // parametrow modelu kinematyki

  BYTE current_servo_algorithm_no[MAX_SERVOS_NR];  // odczytane numery algorytmow
                                                      // serworegulacji
  BYTE next_servo_algorithm_no[MAX_SERVOS_NR];     // wygenerowane numery algorytmow
                                                      // serworegulacji
  BYTE current_servo_parameters_no[MAX_SERVOS_NR]; // odczytane numery zestawow parametrow
                                                      // algorytmow serworegulacji
  BYTE next_servo_parameters_no[MAX_SERVOS_NR];    // wygenerowane numery zestawow parametrow
                                                      // algorytmow serworegulacji
  edp_error error_no;                                 // blad wykryty w EDP
  REPLY_TYPE reply_type;                              // typ odpowiedzi EDP

  robot_ECP_transmission_data (void);
}; // end: robot_ECP_transmission_data
// ------------------------------------------------------------------------

#include "mp/mp_generator.h"
#include "mp/mp_task.h"
#include "mp/mp_robot.h"

#endif
