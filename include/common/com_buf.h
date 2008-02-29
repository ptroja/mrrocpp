// ------------------------------------------------------------------------
// Plik:				com_buf.h
// System:		QNX/MRROC++   v. 6.3
// Opis:			Struktury danych do komunikacji miedzyprocesowej
// Modyfikacja:	Komendy do okien Trajectory Render, Force Create Trajectory
// Jej autor:		tkornuta
// Data:			29.11.2006
// ------------------------------------------------------------------------

#ifndef __COM_BUF_H
#define __COM_BUF_H

#include "common/typedefs.h"
#include "common/impconst.h"

enum SERVO_COMMAND {MOVE, READ, SYNCHRONISE, SERVO_ALGORITHM_AND_PARAMETERS};

// Typ polecenia przesylanego z MP do ECP
// NEXT_STATE // by Y - the next ECP state
enum MP_COMMAND {INVALID_COMMAND, START_TASK, NEXT_POSE, END_MOTION, NEXT_STATE, STOP};

// Typ odpowiedzi ECP na polecenie od MP
enum ECP_REPLY { INCORRECT_MP_COMMAND, ERROR_IN_ECP,
    ECP_ACKNOWLEDGE, TASK_TERMINATED };

enum POSE_SPECIFICATION { INVALID_END_EFFECTOR, FRAME, XYZ_ANGLE_AXIS, XYZ_EULER_ZYZ,
			 JOINT, MOTOR, POSE_FORCE_TORQUE_AT_FRAME };

// Rodzaje odpowiedzi UI do ECP oraz polecen z UI (wcisniecie przycisku).
const BYTE INVALID_REPLY = 0, NEXT = 1, QUIT = 2, ANSWER_YES = 3,
         ANSWER_NO = 4, FILE_LOADED = 5, FILE_SAVED = 6,

         	// Rozkazy z okna Force Control.
		FC_ADD_MACROSTEP = 7, FC_CALIBRATE_SENSOR = 8,
		FC_CHANGE_CONTROL = 9, FC_MOVE_ROBOT = 10,
		FC_SAVE_TRAJECTORY = 11, FC_NEW_TRAJECTORY = 12,
		FC_EXIT = 13, FC_GET_DATA =14,

		// Rozkazy z okna Trajectory Render.
		TR_LOAD_TRAJECTORY = 16, TR_PAUSE_MOVE = 17,
		TR_START_MOVE = 18, TR_STOP_MOVE = 19, TR_EXIT = 20,
		TR_ZERO_POSITION = 21, TR_SAVE_READINGS = 22,
		TR_CALIBRATE_DIGITAL_SCALES_SENSOR = 23,
		TR_CALIBRATE_FORCE_SENSOR = 24, TR_TRY_MOVE_AGAIN = 25,

		// Odpowiedzi z okna z opcjami
		OPTION_ONE = 26, OPTION_TWO = 27, OPTION_THREE = 28, OPTION_FOUR = 29,

             	// Rozkazy z okna MAM_wnd_manual_moves_automatic_measures.
		MAM_START = 30, MAM_STOP = 31, MAM_CLEAR = 32, MAM_SAVE = 33,
		MAM_EXIT = 34, MAM_CALIBRATE =35;

// Dlugosc komunikatu przesylanego z ECP lub MP do UI
#define MSG_LENGTH 60

// Przesylka z ECP do UI
struct ECP_message {
	msg_header_t hdr;
	BYTE ecp_message;   // typ polecenia
	ROBOT_ENUM robot_name; // by Y
	BYTE nr_of_options; // by Y ilosc dostepnych opcji - od 2 do 4 - dla trybu CHOOSE_OPTION
	union{
		char string[MSG_LENGTH];    // komentarz do polecenia
		struct{	// Robot positions + Sensor readings
			double robot_position[MAX_SERVOS_NR];
			double sensor_reading[MAX_SERVOS_NR];
			} RS;
		struct{	// Robot positions + 2 * (Sensor readings)
			double robot_position[MAX_SERVOS_NR];
			double digital_scales_sensor_reading[6];
			double force_sensor_reading[6];
			} R2S;
		struct{	// Robot positions + Sensor readings + Measure number
			double robot_position[MAX_SERVOS_NR];
			double sensor_reading[6];
			int measure_number;
			} MAM;
		};
	};

// Odpowiedz UI do ECP
struct UI_reply {
   msg_header_t hdr;
   BYTE reply;
   int integer_number;
   double double_number;
   double coordinates[MAX_SERVOS_NR];
   char path[80];
   char filename[20];
};

// Przesylka z UI do ECP
struct UI_ECP_message{
	msg_header_t hdr;
	BYTE command ;
	union{
		// nazwa pliku
		char filename[100];
		// czas ruchu
		int motion_time;
		// (okreslenie osi 1..6) && (+/- lewo/prawo)
		short move_type;
		// zmiana rodzaju sterowania
		POSE_SPECIFICATION ps;
		}; // koniec unii
	};

// by Y - do komuniakcji vsp z edp
// Przesylka z VSP do EDP
struct VSP_EDP_message {
   msg_header_t hdr;
   char vsp_name[20];
   short konfigurowac;
};

// Odpowiedz EDP do VSP
struct EDP_VSP_reply {
   unsigned long servo_step;       // by Y numer kroku servo
   double current_present_XYZ_ZYZ_arm_coordinates[6];   // aktualne wspolrzedne XYZ +
   double force[6];
	short force_reading_status; // informacja o odczycie sil
	// EDP_FORCE_SENSOR_OVERLOAD lub EDP_FORCE_SENSOR_READING_ERROR
	// EDP_FORCE_SENSOR_READING_CORRECT

};

// koniec by Y

// #############################################################################################################################

// typ wyliczeniowy przeniesiony na poczatek - bedzie uzywany w trajectory_description    (by Jarosz)

// !!! UWAGA !!!//
// POWIAZANY Z C_MOTOR C_JOINT etc. UWAZAC NA INDEKSY

// Rodzaje polecen ECP dla UI:
const BYTE C_INVALID_END_EFFECTOR = 0, C_FRAME =1, C_XYZ_ANGLE_AXIS = 2,
               C_XYZ_EULER_ZYZ = 3, C_JOINT = 4, C_MOTOR = 5, YES_NO = 6,
               DOUBLE_NUMBER = 7, INTEGER_NUMBER = 8, SAVE_FILE = 9,
               LOAD_FILE = 10, MESSAGE = 11, OPEN_FORCE_SENSOR_MOVE_WINDOW = 12,
               OPEN_TRAJECTORY_REPRODUCE_WINDOW = 13, TR_REFRESH_WINDOW = 14,
               TR_DANGEROUS_FORCE_DETECTED = 15, CHOOSE_OPTION = 16,
               MAM_OPEN_WINDOW = 17, MAM_REFRESH_WINDOW = 18;

// -----------------------------------------------------------------------------------------------------------------------------
struct trajectory_description { // Opis trajektorii do interpolacji dowolnego typu

   POSE_SPECIFICATION arm_type;   // added by Jarosz - do obslugi wszystkich reprezentacji

   int interpolation_node_no;     // Liczba wezlow przy interpolacji
   int internode_step_no;           // Liczba krokow dla jednego przedzialu
   int value_in_step_no;            // Krok, w ktorym zwracana jest odczytana pozycja
   double coordinate_delta[MAX_SERVOS_NR];    // Zadany przyrost polozenia i orientacji
};

// ############################################################################


// Rodzaje procesow w systemie MRROC++
const word16 UNKNOWN_PROCESS_TYPE = 0;
const word16 EDP = 1;
const word16 ECP = 2;
const word16 MP  = 3;
const word16 VSP = 4;
const word16 UI  = 5;

// by Y define dla mozliwych wartosci set_type i get_type
#define CONTROLLER_STATE_DV                     0x08
#define ARM_DV                                  0x04
#define RMODEL_DV                               0x02
#define OUTPUTS_DV                              0x01
#define NOTHING_DV                              0x00
// end by Y

/* numery bledow generowanych w EDP */
#define OK                                      0x0000000000000000ULL

#define INVALID_INSTRUCTION_TYPE                0x0100000000000000ULL
#define INVALID_REPLY_TYPE                      0x0200000000000000ULL
#define INVALID_SET_RMODEL_TYPE                 0x0300000000000000ULL
#define INVALID_GET_RMODEL_TYPE                 0x0400000000000000ULL
#define ERROR_IN_RMODEL_REQUEST                 0x0500000000000000ULL
#define INVALID_HOMOGENEOUS_MATRIX              0x0600000000000000ULL

#define QUERY_EXPECTED                          0x1000000000000000ULL
#define QUERY_NOT_EXPECTED                      0x1100000000000000ULL
#define NO_VALID_END_EFFECTOR_POSE              0x1200000000000000ULL
#define INVALID_MOTION_TYPE                     0x1300000000000000ULL
#define INVALID_MOTION_PARAMETERS               0x1400000000000000ULL
#define INVALID_SET_END_EFFECTOR_TYPE           0x1500000000000000ULL
#define INVALID_GET_END_EFFECTOR_TYPE           0x1600000000000000ULL
#define STRANGE_GET_ARM_REQUEST                 0x1700000000000000ULL
/* -------------------------------------------------------------------- */
/* Kody bledow programowych (niefatalne) przekroczenia zakresow         */
/* -------------------------------------------------------------------- */

// Przekroczenie zakresow ruchu walow silnikow
#define BEYOND_UPPER_LIMIT_AXIS_0               0x2000000000000000ULL
#define BEYOND_UPPER_LIMIT_AXIS_1               0x2100000000000000ULL
#define BEYOND_UPPER_LIMIT_AXIS_2               0x2200000000000000ULL
#define BEYOND_UPPER_LIMIT_AXIS_3               0x2300000000000000ULL
#define BEYOND_UPPER_LIMIT_AXIS_4               0x2400000000000000ULL
#define BEYOND_UPPER_LIMIT_AXIS_5               0x2500000000000000ULL
#define BEYOND_UPPER_LIMIT_AXIS_6               0x2600000000000000ULL
#define BEYOND_UPPER_LIMIT_AXIS_7               0x2700000000000000ULL

#define BEYOND_LOWER_LIMIT_AXIS_0               0x2800000000000000ULL
#define BEYOND_LOWER_LIMIT_AXIS_1               0x2900000000000000ULL
#define BEYOND_LOWER_LIMIT_AXIS_2               0x2A00000000000000ULL
#define BEYOND_LOWER_LIMIT_AXIS_3               0x2B00000000000000ULL
#define BEYOND_LOWER_LIMIT_AXIS_4               0x2C00000000000000ULL
#define BEYOND_LOWER_LIMIT_AXIS_5               0x2D00000000000000ULL
#define BEYOND_LOWER_LIMIT_AXIS_6               0x2E00000000000000ULL
#define BEYOND_LOWER_LIMIT_AXIS_7               0x2F00000000000000ULL

// bledy przekroczenia zakresu we wspolrzednych wewnetrznych
#define BEYOND_UPPER_D0_LIMIT                   0x3000000000000000ULL
#define BEYOND_UPPER_THETA1_LIMIT               0x3100000000000000ULL
#define BEYOND_UPPER_THETA2_LIMIT               0x3200000000000000ULL
#define BEYOND_UPPER_THETA3_LIMIT               0x3300000000000000ULL
#define BEYOND_UPPER_THETA4_LIMIT               0x3400000000000000ULL
#define BEYOND_UPPER_THETA5_LIMIT               0x3500000000000000ULL
#define BEYOND_UPPER_THETA6_LIMIT               0x3600000000000000ULL
#define BEYOND_UPPER_THETA7_LIMIT               0x3700000000000000ULL


#define BEYOND_LOWER_D0_LIMIT                   0x3800000000000000ULL
#define BEYOND_LOWER_THETA1_LIMIT               0x3900000000000000ULL
#define BEYOND_LOWER_THETA2_LIMIT               0x3A00000000000000ULL
#define BEYOND_LOWER_THETA3_LIMIT               0x3B00000000000000ULL
#define BEYOND_LOWER_THETA4_LIMIT               0x3C00000000000000ULL
#define BEYOND_LOWER_THETA5_LIMIT               0x3D00000000000000ULL
#define BEYOND_LOWER_THETA6_LIMIT               0x3E00000000000000ULL
#define BEYOND_LOWER_THETA7_LIMIT               0x3F00000000000000ULL

#define OUT_OF_WORKSPACE                        0x4100000000000000ULL
#define SINGULAR_POSE                           0x4200000000000000ULL

// bledy standardowych funkcji matematycznych: argument spoza dziedziny
#define ACOS_DOMAIN_ERROR                       0x4300000000000000ULL
#define ASIN_DOMAIN_ERROR                       0x4400000000000000ULL
#define ATAN2_DOMAIN_ERROR                      0x4500000000000000ULL
#define SQRT_DOMAIN_ERROR                       0x4600000000000000ULL
#define UNKNOWN_MATH_ERROR                      0x4700000000000000ULL

#define UNKNOWN_INSTRUCTION                     0x4800000000000000ULL
#define NOT_IMPLEMENTED_YET                     0x4900000000000000ULL

#define NOT_YET_SYNCHRONISED                    0x5200000000000000ULL
#define ALREADY_SYNCHRONISED                    0x5300000000000000ULL
#define UNKNOWN_SYNCHRO_ERROR                   0x5400000000000000ULL
#define INVALID_KINEMATIC_MODEL_NO              0x5500000000000000ULL
#define INVALID_KINEMATIC_CORRECTOR_NO          0x5600000000000000ULL
#define EDP_UNIDENTIFIED_ERROR                  0x5700000000000000ULL


#define NOT_A_NUMBER_JOINT_VALUE_D0             0x6000000000000000ULL
#define NOT_A_NUMBER_JOINT_VALUE_THETA1         0x6100000000000000ULL
#define NOT_A_NUMBER_JOINT_VALUE_THETA2         0x6200000000000000ULL
#define NOT_A_NUMBER_JOINT_VALUE_THETA3         0x6300000000000000ULL
#define NOT_A_NUMBER_JOINT_VALUE_THETA4         0x6400000000000000ULL
#define NOT_A_NUMBER_JOINT_VALUE_THETA5         0x6500000000000000ULL
#define NOT_A_NUMBER_JOINT_VALUE_THETA6         0x6600000000000000ULL
#define NOT_A_NUMBER_JOINT_VALUE_THETA7         0x6700000000000000ULL


/* bledy wykrywany przez SERVO_GROUP */
#define SERVO_ERROR_IN_PASSIVE_LOOP             0x0004000000000000ULL
#define UNIDENTIFIED_SERVO_COMMAND              0x0008000000000000ULL   // otrzymano wadliwe polecenie z EDP_MASTER
#define SERVO_ERROR_IN_PHASE_1                  0x000C000000000000ULL
#define SERVO_ERROR_IN_PHASE_2                  0x0010000000000000ULL

/* bledy wykryte przy synchronizacji serwomechanizmow */
#define SYNCHRO_SWITCH_EXPECTED                 0x0014000000000000ULL
#define SYNCHRO_ERROR                           0x0018000000000000ULL
#define SYNCHRO_DELAY_ERROR                     0x001C000000000000ULL

// Klasy bledow
#define NEW_MESSAGE                              0x1ULL
#define SYSTEM_ERROR                             0x2ULL
#define FATAL_ERROR                              0x3ULL
#define NON_FATAL_ERROR                          0x4ULL

// Bledy szczegolowe generowane przez ECP i MP
#define INVALID_MP_COMMAND                       0x1ULL
#define INVALID_POSE_SPECIFICATION               0x2ULL
#define INVALID_RMODEL_TYPE                      0x3ULL
#define INVALID_ECP_COMMAND                      0x4ULL
#define INVALID_EDP_REPLY                        0x5ULL
#define ECP_ERRORS                               0x6ULL
#define INVALID_COMMAND_TO_EDP                   0x7ULL
#define ECP_UNIDENTIFIED_ERROR                   0x8ULL
#define MP_UNIDENTIFIED_ERROR                    0x9ULL
#define EDP_ERROR                                0xAULL
#define NON_EXISTENT_DIRECTORY                   0xBULL
#define NON_EXISTENT_FILE                        0xCULL
#define READ_FILE_ERROR                          0xDULL
#define NON_TRAJECTORY_FILE                      0xEULL
#define NON_COMPATIBLE_LISTS                     0xFULL
#define ECP_STOP_ACCEPTED                       0x10ULL
#define MAX_ACCELERATION_EXCEEDED               0x11ULL
#define MAX_VELOCITY_EXCEEDED                   0x12ULL
#define NOT_ENOUGH_MEMORY                       0x13ULL
#define INVALID_TIME_SPECIFICATION              0x14ULL
#define INVALID_ECP_PULSE_IN_MP_START_ALL       0x16ULL
#define INVALID_ECP_PULSE_IN_MP_EXECUTE_ALL     0x17ULL
#define INVALID_ECP_PULSE_IN_MP_TERMINATE_ALL   0x18ULL

// Bledy szczegolowe generowane przez ECP i MP - DO VSP
// Bledy SYSTEM_ERROR
#define CANNOT_SPAWN_VSP                        0x20ULL
#define CANNOT_LOCATE_DEVICE                    0x21ULL
#define CANNOT_READ_FROM_DEVICE                 0x22ULL
#define CANNOT_WRITE_TO_DEVICE                  0x23ULL
#define DEVICE_ALREADY_EXISTS                   0x24ULL
// Bledy FATAL_ERROR
#define BAD_VSP_REPLY                           0x25ULL
// Bledy NON_FATAL_ERROR
#define INVALID_VSP_REPLY                       0x26ULL
// Bledy inne
#define VSP_UNIDENTIFIED_ERROR                  0x27ULL
// Bledy generowane w generatorze trajektorii
#define DANGEROUS_FORCE_DETECTED                0x28ULL
#define SAVE_FILE_ERROR                         0x29ULL
#define NAME_ATTACH_ERROR                       0x2AULL

// Bledy szczegolowe generowane przez VSP
// Bledy SYSTEM_ERROR
#define DISPATCH_ALLOCATION_ERROR                0x1ULL
#define DEVICE_EXISTS                            0x2ULL
#define DEVICE_CREATION_ERROR                    0x3ULL
#define DISPATCH_LOOP_ERROR                      0x4ULL
// Bledy FATAL_ERROR
#define SENSOR_NOT_CONFIGURED                    0x5ULL
#define READING_NOT_READY                        0x6ULL
// Bledy NON_FATAL_ERROR
#define INVALID_COMMAND_TO_VSP                   0x7ULL
// Bledy FATAL ERROR - bufor cykliczny.
#define CYCLIC_BUFFER_PARSE_ERROR               0x08ULL
#define CYCLIC_BUFFER_UNDERRUN                  0x09ULL
#define CYCLIC_BUFFER_OVERFLOW                  0x10ULL


#define EDP_SYSTEM_ERROR                         0x1ULL
#define ECP_SYSTEM_ERROR                         0x2ULL
#define MP_SYSTEM_ERROR                          0x3ULL
#define EDP_FATAL_ERROR                          0x4ULL
#define EDP_NON_FATAL_ERROR                      0x5ULL
#define ECP_ERROR                                0x6ULL
#define MP_ERROR                                 0x7ULL


// crs - bledy dotyczace CRS - znajdywania rozwiazania dla kostki Rubika
#define RCS_INVALID_STATE                       0x10ULL
#define RCS_EXCEPTION                           0x11ULL


enum GRIPPER_STATE_ENUM {
	GRIPPER_START_STATE,
	GRIPPER_EXPAND_STATE,
	GRIPPER_NARROW_STATE,
	GRIPPER_BLOCKED_AFTER_EXPAND_STATE,
	GRIPPER_BLOCKED_AFTER_NARROW_STATE,
	GRIPPER_BLOCKED_STATE
};

enum INSTRUCTION_TYPE    { INVALID, SET, GET, SET_GET, SYNCHRO, QUERY };
enum RMODEL_SPECIFICATION { INVALID_RMODEL, TOOL_FRAME, TOOL_XYZ_ANGLE_AXIS, TOOL_XYZ_EULER_ZYZ, TOOL_AS_XYZ_EULER_ZY, ARM_KINEMATIC_MODEL, SERVO_ALGORITHM };
enum MOTION_TYPE	{ ABSOLUTE, RELATIVE,
								PF_XYZ_ANGLE_AXIS_ABSOLUTE_POSE, PF_JOINTS_ABSOLUTE_POSITION, PF_MOTORS_ABSOLUTE_POSITION,
								PF_XYZ_ANGLE_AXIS_RELATIVE_POSE, PF_JOINTS_RELATIVE_POSITION, PF_MOTORS_RELATIVE_POSITION,
								PF_VELOCITY};
enum REPLY_TYPE            {ERROR, ACKNOWLEDGE, SYNCHRO_OK, ARM, RMODEL, INPUTS,
                                  ARM_RMODEL, ARM_INPUTS, RMODEL_INPUTS, ARM_RMODEL_INPUTS, CONTROLLER_STATE };

enum BEHAVIOUR_SPECIFICATION	{UNGUARDED_MOTION, GUARDED_MOTION, CONTACT};


/*--------------------------------------------------------------------------*/
struct edp_master_command // wzorzec polecenia przesylanego z EDP_MASTER do SERVO_GROUP
{
   SERVO_COMMAND instruction_code;        // kod polecenia, ktore ma zreallizowac proces
                                           // SERVO_GROUP
   BYTE address_byte;                         // bajt do obliczania dlugosci rozkazu
   union {
     struct {
        WORD number_of_steps;                     // liczba krokow, w ktorej ma byc wykonany makrokrok
        WORD return_value_in_step_no;           // liczba krokow, po ktorej ma byc przeslana informacja
                                             // do EDP_MASTER (za posrednictwem READING_BUFFER) o
                                             // uprzednio zrealizowanym polozeniu (po zrealizowaniu
                                             // k krokow SERVO ma polozenie uzyskane w k-1 kroku)
        double macro_step[MAX_SERVOS_NR]; // wielkosc makrokroku (wartosc zadana makrokroku - przyrost)
        double abs_position [MAX_SERVOS_NR]; // by Y - zadana pozycja absolutna na koniec makrokroku
    //    BYTE address_byte;               // bajt do obliczania dlugosci rozkazu
     } move;
     struct {
        BYTE servo_algorithm_no[MAX_SERVOS_NR];   // numery algorytmow serworegulacji
        BYTE servo_parameters_no[MAX_SERVOS_NR]; // numery zestawu parametrow algorytmow serworegulacji
    //    BYTE address_byte;               // bajt do obliczania dlugosci rozkazu
     } servo_alg_par;
   } parameters;
}; // end: edp_master_command
/*--------------------------------------------------------------------------*/


/*--------------------------------------------------------------------------*/
struct edp_error // struktura zawierajaca zakodowana przyczyne bledu
{
   uint64_t error0;
   uint64_t error1;
};
/*--------------------------------------------------------------------------*/


/*--------------------------------------------------------------------------*/
struct servo_group_reply // wzorzec odpowiedzi przesylanej z SERVO_GROUP do EDP_MASTER
{
 edp_error error;                           // struktura zawierajaca zakodowana przyczyne bledu
 double position[MAX_SERVOS_NR];   // przyrost polozenia walu silnika
                                         // osiagniety od ostatniego odczytu
 double abs_position[MAX_SERVOS_NR];   // by Y - bezwzgledna pozycja stawow w radianach
 word16 PWM_value[MAX_SERVOS_NR]; // wartosci zadane wypelnienia PWM -- zazwyczaj zbedne
 word16 current[MAX_SERVOS_NR];    // prad sterujacy -- zazwyczaj zbedne
 BYTE algorithm_no[MAX_SERVOS_NR];// numery uzywanych algorytmow regulacji
 BYTE algorithm_parameters_no[MAX_SERVOS_NR];
 short gripper_reg_state; // stan w ktorym znajduje sie regulator chwytaka
                                     // numery uzywanych zestawow parametrow algorytmow regulacji
}; // end: servo_group_reply
/*--------------------------------------------------------------------------*/

/****************************** c_buffer *******************************/


typedef union { // rmodel
	struct {
		frame_tab tool_frame_m; // trojscian narzedzia wzgledem kolnierza
	// 	BYTE address_byte;         // bajt do obliczania dlugosci rozkazu
	} tool_frame_def;
	struct {
		double tool_coordinates[6];   // XYZ + orientacja narzedzia wzgledem kolnierza
	// 	BYTE address_byte;               // bajt do obliczania dlugosci rozkazu
	} tool_coordinate_def;
	struct {
		BYTE kinematic_model_no;               // numer zestawu parametrow modelu kinematyki

	// 	BYTE address_byte;               // bajt do obliczania dlugosci rozkazu
	} kinematic_model;
	struct {
		BYTE servo_algorithm_no[MAX_SERVOS_NR];   // numery algorytmow serworegulacji
		BYTE servo_parameters_no[MAX_SERVOS_NR]; // numery zestawu parametrow algorytmow serworegulacji
// 		BYTE address_byte;               // bajt do obliczania dlugosci rozkazu
	} servo_algorithm;
} c_buffer_rmodel;


typedef union { // arm
		struct {
 			int command;        // wariat zapytania o get_state
		} get_state_def;
		struct {
 			frame_tab arm_frame_m;        // trojscian koncowki wzgledem ukladu bazowego
 			double gripper_coordinate; // stopien rozwarcia chwytaka
 		// 	BYTE address_byte;                // bajt do obliczania dlugosci rozkazu
		} frame_def;
		struct {
 			double arm_coordinates[MAX_SERVOS_NR];    // XYZ + orientacja koncowki wzgledem ukladu bazowego
 			double desired_torque[MAX_SERVOS_NR]; // zadany moment dla dunga
 		// 	BYTE address_byte;                // bajt do obliczania dlugosci rozkazu
 			double gripper_coordinate; // stopien rozwarcia chwytaka
		} coordinate_def;
	 	struct { // by Y do sterowania pozycyjno silowego
	 		double inertia[6], reciprocal_damping[6];
			double position_velocity [MAX_SERVOS_NR];
			double force_xyz_torque_xyz[6];
			BEHAVIOUR_SPECIFICATION behaviour[6];
			double gripper_coordinate; // stopien rozwarcia chwytaka
			// 	BYTE address_byte;                // bajt do obliczania dlugosci rozkazu
	 	} pose_force_torque_at_frame_def; // end by Y
		struct {
			char text[MAX_TEXT]; // MAC7
			char prosody[MAX_PROSODY]; // MAC7
		// 	BYTE address_byte;
		} text_def;
} c_buffer_arm;

#define POSE_SV_AX 0
#define FORCE_SV_AX 1

struct c_buffer {

    struct _pulse hdr;

   	INSTRUCTION_TYPE instruction_type; // typ instrukcji: SET, GET, SET_GET, SYNCHRO, QUERY

   	BYTE set_type;                            // typ instrukcji set: ARM/RMODEL/OUTPUTS
   	BYTE get_type;                            // typ instrukcji get: ARM/RMODEL/INPUTS
   	RMODEL_SPECIFICATION set_rmodel_type;   // sposob zdefiniowania narzedzia przy jego zadawaniu:
                                         // TOOL_FRAME / TOOL_XYZ_EULER_ZYZ / TOOL_XYZ_ANGLE_AXIS / TOOL_AS_XYZ_EULER_ZY /
                                         // ARM_KINEMATIC_MODEL / SERVO_ALGORITHM
   	RMODEL_SPECIFICATION get_rmodel_type;   // sposob zdefiniowania narzedzia przy jego odczycie:
                                         // TOOL_FRAME / TOOL_XYZ_EULER_ZYZ / TOOL_XYZ_ANGLE_AXIS / TOOL_AS_XYZ_EULER_ZY /
                                         // ARM_KINEMATIC_MODEL / SERVO_ALGORITHM
   	POSE_SPECIFICATION set_arm_type;    // sposob zdefiniowania polozenia zadanego
                                         // koncowki: MOTOR / JOINT /
                                         // FRAME / XYZ_EULER_ZYZ / XYZ_ANGLE_AXIS /
   	POSE_SPECIFICATION get_arm_type;    // sposob zdefiniowania polozenia odcztanego
                                         // koncowki: MOTOR / JOINT /
                                         // FRAME / XYZ_EULER_ZYZ / XYZ_ANGLE_AXIS
   	WORD output_values;                     // wartosci wyjsc binarnych
   	BYTE address_byte;                       // bajt do obliczania dlugosci rozkazu

 	MOTION_TYPE motion_type;        // sposob zadania ruchu: ABSOLUTE/RELATIVE
	WORD motion_steps;                // liczba krokow ruchu zadanego (makrokroku)
	WORD value_in_step_no;           // liczba krokow pierwszej fazy ruchu, czyli
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

	c_buffer_rmodel rmodel;
	c_buffer_arm arm;

	c_buffer (void);// by W odkomentowane
};
/****************************** c_buffer *******************************/


/************************ r_buffer ****************************/


typedef union {   // rmodel
	struct {
		frame_tab tool_frame_m;         // Macierz reprezentujaca narzedzie
 				                                    // wzgledem konca lancucha kinematycznego
//		BYTE address_byte;             // bajt do obliczania dlugosci odpowiedzi
	} tool_frame_def;
	struct {
		double tool_coordinates[6]; // XYZ + orientacja narzedzia wzgledem kolnierza
//		BYTE address_byte;             // bajt do obliczania dlugosci odpowiedzi
	} tool_coordinate_def;
	struct {
		BYTE kinematic_model_no;       // numer modelu kinematyki

//		BYTE address_byte;               // bajt do obliczania dlugosci rozkazu
	} kinematic_model;
	struct {
		BYTE servo_algorithm_no[MAX_SERVOS_NR];   // numery algorytmow serworegulacji
		BYTE servo_parameters_no[MAX_SERVOS_NR]; // numery zestawu parametrow algorytmow serworegulacji
	} servo_algorithm;
} r_buffer_rmodel;

typedef struct {
		bool is_synchronised;        // czy robot jest zsynchronizowany
		bool is_power_on;        // czy wzmacniacze mocy sa zasilane
		bool is_wardrobe_on;        // czy szafa jest wlaczona
		bool is_controller_card_present;        // czy karta kontrolera robota jest w zamontowana w komputerze
		bool is_robot_blocked;        // czy wyzerowana sterowanie na silnikach po awarii sprzetowej
	} controller_state_typedef;


typedef union {   // arm
	struct {
		word16 PWM_value[MAX_SERVOS_NR];               // wartosci zadane wypelnienia PWM -- zazwyczaj zbedne
		word16 current[MAX_SERVOS_NR];                 // prad sterujacy -- zazwyczaj zbedne
		frame_tab arm_frame_m;           // Macierz reprezentujaca koncowke
				                                       // wzgledem bazy manipulator
		short gripper_reg_state; // stan w ktorym znajduje sie regulator chwytaka
		double gripper_coordinate; // stopien rozwarcia chwytaka
		// 	BYTE address_byte;             // bajt do obliczania dlugosci odpowiedzi
	} frame_def;
	struct {
		word16 PWM_value[MAX_SERVOS_NR];               // wartosci zadane wypelnienia PWM -- zazwyczaj zbedne
		word16 current[MAX_SERVOS_NR];                 // prad sterujacy -- zazwyczaj zbedne
		double arm_coordinates[MAX_SERVOS_NR];   // XYZ + orientacja koncowki wzgledem ukladu bazowego
		short gripper_reg_state; // stan w ktorym znajduje sie regulator chwytaka
		double gripper_coordinate; // stopien rozwarcia chwytaka
		// 	BYTE address_byte;             // bajt do obliczania dlugosci odpowiedzi
	} coordinate_def;
	struct { // by Y do sterowania pozycyjno silowego
		word16 PWM_value[MAX_SERVOS_NR];               // wartosci zadane wypelnienia PWM -- zazwyczaj zbedne
		word16 current[MAX_SERVOS_NR];
		frame_tab   beggining_arm_frame_m;        // trojscian koncowki wzgledem ukladu bazowego
		frame_tab   predicted_arm_frame_m;        // trojscian koncowki wzgledem ukladu bazowego
		frame_tab   present_arm_frame_m;        // trojscian koncowki wzgledem ukladu bazowego
		// double pos_xyz_rot_xyz[6];
		double force_xyz_torque_xyz[6];
		short gripper_reg_state; // stan w ktorym znajduje sie regulator chwytaka
		double gripper_coordinate; // stopien rozwarcia chwytaka
	// 	BYTE address_byte;                // bajt do obliczania dlugosci rozkazu
	} pose_force_torque_at_frame_def; // end by Y
	struct{
		int speaking; // czy mowi
		// BYTE address_byte;
	} text_def;
} r_buffer_arm;



struct r_buffer {

	REPLY_TYPE reply_type; // typ odpowiedzi: ERROR, ACKNOWLEDGE, SYNCHRO_OK, ARM, RMODEL, INPUTS,
                                 //           ARM_RMODEL, ARM_INPUTS, RMODEL_INPUTS, ARM_RMODEL_INPUT, CONTROLLER_STATE
					 // by Y - rozszerzone o sile
	edp_error error_no;                     // numer bledu, jezeli wystapil
	RMODEL_SPECIFICATION rmodel_type;   // sposob zdefiniowania narzedzia przy jego odczycie:
                                         // TOOL_FRAME / TOOL_XYZ_EULER_ZYZ / TOOL_XYZ_ANGLE_AXIS / TOOL_AS_XYZ_EULER_ZY /
                                         // ARM_KINEMATIC_MODEL / SERVO_ALGORITHM
	POSE_SPECIFICATION arm_type;    // sposob zdefiniowania polozenia zadanego
                                       // koncowki: MOTOR / JOINT / FRAME
						    // XYZ_EULER_ZYZ / POSE_FORCE_LINEAR / XYZ_ANGLE_AXIS / POSE_FORCE_TORQUE_AT_FRAME
	WORD input_values;                 // wartosci wejsc binarnych
	BYTE analog_input[8];		// wejscie analogowe
	controller_state_typedef controller_state;

	unsigned long servo_step;       // by Y numer kroku servo

   	BYTE address_byte;                 // bajt do obliczania dlugosci odpowiedzi

	word16 PWM_value[MAX_SERVOS_NR];             // wartosci zadane wypelnienia PWM -- zazwyczaj zbedne
	word16 current[MAX_SERVOS_NR];                // prad sterujacy -- zazwyczaj zbedne

	r_buffer_rmodel rmodel;
	r_buffer_arm arm;

	r_buffer (void); // W odkomentowane
};
/************************ r_buffer ****************************/


// ------------------------------------------------------------------------
class ecp_command_buffer {
public:

   c_buffer instruction;     // bufor polecen przysylanych z ECP do EDP

	// zlecenie zmiany stanu skojarzone z NEXT_STATE
	int mp_2_ecp_next_state;
	int mp_2_ecp_next_state_variant; // skojarzone z NEXT_STATE
	char mp_2_ecp_next_state_string[MP_2_ECP_STRING_SIZE]; // skojarzone z NEXT_STATE

   bool is_set_rmodel() const  { return (bool) (instruction.set_type & RMODEL_DV); }; // zmienic narzedzie?
   bool is_set_arm() const   { return (bool) (instruction.set_type & ARM_DV); }; // zmienic polozenie ramienia?
}; // end: class command_buffer
// ------------------------------------------------------------------------

// ------------------------------------------------------------------------
class edp_reply_buffer {
public:
   r_buffer reply_package;           // bufor odpowiedzi wysylanych do ECP
}; // end: class reply_buffer
// ------------------------------------------------------------------------

// ------------------------------------------------------------------------
// Polecenie MP dla ECP
struct MP_COMMAND_PACKAGE {
   msg_header_t hdr;
   MP_COMMAND command;
   ecp_command_buffer mp_package;
};
// ------------------------------------------------------------------------

// ------------------------------------------------------------------------
// Odpowiedz ECP do MP
struct ECP_REPLY_PACKAGE {
   ECP_REPLY reply;
   edp_reply_buffer ecp_reply;
};
// ------------------------------------------------------------------------


/*
// by Y
inline void copy_frame(frame_tab destination_frame,   frame_tab source_frame){
     for (int   column = 0; column < 4; column++)
       for (int row = 0; row < 3; row++)
        destination_frame[column][row] = source_frame[column][row];
}
*/

#endif
