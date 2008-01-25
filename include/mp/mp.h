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
#include <map>
#include <list>

#include "ecp_mp/ecp_mp_task.h"
// Konfigurator.
#include "lib/configurator.h"
#include "lib/mp_timer.h"


enum WAIT_FOR_STOP_ENUM {
	MP_EXIT,
	MP_THROW
};


class mp_robot;
class mp_generator;


// struktura zwracana przez funkcje mp_receive_ecp_pulse
typedef struct {
	uint32_t nd; // deskryptor wezla na ktorym jest powolane ECP
	pid_t ECP_pid;
	bool rt;
	int32_t scoid; // server connection id
	char pulse_code;
	int rcvid;
	uint64_t e; // errno
} mp_receive_ecp_pulse_return_td;


// struktura wykorzystywana przez funkcje mp_receive_pulse
typedef struct {
	_msg_info msg_info;
	_pulse_msg pulse_msg;
	int rcvid;
	uint64_t e;       // Kod bledu systemowego
} mp_receive_pulse_struct_tdef;

// klasa globalna dla calego procesu MP
class mp_task: public ecp_mp_task  {
protected:
	// lsita generatorow dla scheduller'a
    std::list<mp_generator*> gen_list;
    std::list<mp_generator*>::iterator gen_list_iterator;
	
	bool all_gen_sets_waiting_for_ECP_pulse;

public:
	static name_attach_t *mp_trigger_attach;
	static name_attach_t *mp_attach;
	
	// mapa wszystkich robotow
	static std::map <ROBOT_ENUM, mp_robot*> robot_m;

        int32_t ui_scoid; // server connection id
	  char ui_pulse_code; // kod pulsu ktory zostal wyslany przez ECP w celu zgloszenia gotowosci do komunikacji (wartosci w impconst.h)
 	  bool ui_new_pulse; // okresla czy jest nowy puls
		
	// KONSTRUKTORY
	mp_task(void);
	virtual ~mp_task(void);
	
	// METODY
	// funkcja odbierajaca pulsy z ECP
	
	// -------------------------------------------------------------------
	// inicjacja polaczen, rejestracja nazwy MP, odszukanie UI, SR by Y&W
	// -------------------------------------------------------------------
	void mp_initialize_communication (void); 

	// oczekiwanie na puls z ECP
	enum MP_RECEIVE_PULS_ENUM {
		WITH_TIMEOUT,
		WITHOUT_TIMEOUT
	};
	
	enum WAIT_FOR_NEW_PULSE_ENUM {
		NEW_ECP_PULSE,
		NEW_UI_PULSE,
		NEW_UI_OR_ECP_PULSE
	};

	bool set_next_ecps_state (int l_state, int l_variant, char* l_string, int number_of_robots, ... );
	bool send_end_motion_to_ecps (int number_of_robots, ... );
	bool run_ext_empty_gen (bool activate_trigger, int number_of_robots, ... );
	bool run_ext_empty_gen_for_set_of_robots_and_wait_for_task_termin_mess_of_another_set_of_robots
		(int number_of_robots_to_move, int number_of_robots_to_wait_for_task_termin, ... );
	bool wait_ms (int _ms_delay); // zamiast delay

	
	int mp_receive_pulse (mp_receive_pulse_struct_tdef* outputs, MP_RECEIVE_PULS_ENUM tryb);
	int check_and_optional_wait_for_new_pulse (mp_receive_pulse_struct_tdef* outputs, 
		WAIT_FOR_NEW_PULSE_ENUM process_mode, MP_RECEIVE_PULS_ENUM desired_wait_mode);
	int mp_wait_for_ui_name_open(void);
	
	// mp_receive_ecp_pulse_return_td mp_receive_ecp_pulse (int tryb);
	// oczekwianie na name_open do kanalu do przesylania pulsow miedzy ECP i MP
	// int mp_wait_for_name_open_ecp_pulse (mp_receive_pulse_struct_tdef* outputs, uint32_t nd, pid_t ECP_pid);
	int mp_wait_for_name_open_ecp_pulse(mp_receive_pulse_struct_tdef* outputs);
	
	// Oczekiwanie na zlecenie START od UI
	void wait_for_start (void);// by Y&W

	// Oczekiwanie na zlecenie STOP od UI
	void wait_for_stop ( WAIT_FOR_STOP_ENUM tryb);// by Y&W dodany tryb
	
	// Wystartowanie wszystkich ECP
	void start_all (std::map <ROBOT_ENUM, mp_robot*>& _robot_m);
	
	// Zatrzymanie wszystkich ECP
	void terminate_all (std::map <ROBOT_ENUM, mp_robot*>& _robot_m );
	
	// Wyslanie rozkazu do wszystkich ECP
	void execute_all (std::map <ROBOT_ENUM, mp_robot*>& _robot_m);
	
	// funkcja odbierajaca pulsy z UI wykorzystywana w MOVE
	bool mp_receive_ui_pulse (std::map <ROBOT_ENUM, mp_robot*>& _robot_m, short* trigger);
	
	// funkcja odbierajaca pulsy z UI lub ECP wykorzystywana w MOVE
	bool mp_receive_ui_or_ecp_pulse (std::map <ROBOT_ENUM, mp_robot*>& _robot_m, mp_generator& the_generator );
	
	// Funkcja ruchu
	bool Move ( mp_generator& the_generator );
	
	// Funkcja oczekiwania na spelnienie warunku poczatkowego
	              

	// dla scheduller'a
	bool add_gen (mp_generator* gen_l);
	// not used anymore (ptrojane)
	//bool rm_gen (mp_generator* gen_l);
	bool clear_gen_list (void);
	
	bool scheduller_run (void);
	
	// obsluga sygnalu
	virtual void catch_signal_in_mp_task(int sig);
	
	// Zatrzymanie wszystkich ECP
	static void kill_all_ECP (std::map <ROBOT_ENUM, mp_robot*>& _robot_m);
	
	// utworzenie robotow
	bool create_robots(void);
	
	// methods for MP template to redefine in concrete classes
	virtual void task_initialization(void);
	virtual void main_task_algorithm(void);
};


mp_task* return_created_mp_task (void);


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

	double MPtoECP_inertia[6], MPtoECP_reciprocal_damping[6], MPtoECP_stiffness[6];
	double MPtoECP_position_velocity[MAX_SERVOS_NR], MPtoECP_stiffness_base_position[MAX_SERVOS_NR], MPtoECP_force_xyz_torque_xyz[6]; 
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

// ==================== KLASY BAZOWE ============================



// ---------------------------------------------------------------
class mp_robot {
   // Klasa bazowa dla robotow (klasa abstrakcyjna)
   // Kazdy robot konkretny (wyprowadzony z klasy bazowej)
   // musi zawierac pola danych (skladowe) dotyczace
   // ostatnio zrealizowanej pozycji oraz pozycji zadanej
protected:
  MP_COMMAND_PACKAGE mp_command;      // Bufor z rozkazem dla ECP
                                      // - uzytkownik nie powinien z tego korzystac
  ECP_REPLY_PACKAGE ecp_reply;        // Bufor z odpowiedzia z ECP
                                      // - uzytkownik nie powinien z tego korzystac

mp_task* mp_object;


 public:
 sr_ecp* sr_ecp_msg;    // by Y - Wskaznik na obiekt do komunikacji z SR

  ROBOT_ENUM robot_name;
  uint32_t nd; // deskryptor wezla na ktorym jest powolane ECP
  pid_t ECP_pid;
  int32_t scoid; // server connection id
  int ECP_fd;	// by Y&W do komunikacji, zamiast ECP_pid, ktore wystrczalo w QNX 4
  char pulse_code; // kod pulsu ktory zostal wyslany przez ECP w celu zgloszenia gotowosci do komunikacji (wartosci w impconst.h)
  bool new_pulse; // okresla czy jest nowy puls
  bool robot_new_pulse_checked; // okresla czy czy nowy puls zostal juz uwzgledniony w generatorze
   bool communicate; // okresla czy robot ma byc obslugiwany w execute_all
   
  robot_ECP_transmission_data ecp_td; // Obraz danych robota wykorzystywanych przez generator
                                      // - do uzytku uzytkownika (generatora)

  mp_robot (ROBOT_ENUM l_robot_name, const char* _section_name, mp_task* mp_object_l);

  class MP_error {  // Klasa obslugi bledow robotow
    public:
      const uint64_t error_class;
      const uint64_t mp_error;
      MP_error (uint64_t err0, uint64_t err1);
  }; // end: class MP_error

  virtual void execute_motion (void);
    // Zlecenie wykonania ruchu przez robota
    // (realizowane przez klase konkretna):
    // na poziomie MP jest to polecenie dla ECP.

  virtual void terminate_ecp (void);
    // Zlecenie zakonczenia ruchu przez robota
    // (realizowane przez klase konkretna):
    // na poziomie MP jest to polecenie dla ECP.
  virtual void start_ecp ( void );

  virtual void create_command ( void );
    // wypelnia bufor wysylkowy do EDP na podstawie danych zawartych w swych skladowych
	// Ten bufor znajduje sie w robocie

virtual void create_next_pose_command (void) = 0;

  virtual void get_reply ( void ) = 0;

    // pobiera z pakietu przeslanego z EDP informacje i wstawia je do odpowiednich swoich skladowych
}; // end: class mp_robot
// ---------------------------------------------------------------

// ---------------------------------------------------------------
class mp_generator {
    // Klasa bazowa dla generatorow trajektorii (klasa abstrakcyjna)
    // Sluzy zarowno do wyznaczania nastepnej wartosci zadanej jak i
    // sprawdzania spelnienia warunku koncowego    

protected:
	sr_ecp* sr_ecp_msg;    // by Y - Wskaznik na obiekt do komunikacji z SR

	int idle_step_counter; // Licznik jalowych krokow sterowania (bez wykonywania ruchu)

public:

	// zbior obejmujacy mozliwe stany obiektu klasy generator_set
	enum GEN_SET_PHASE {
		BEFORE_FIRST_STEP,
		AFTER_STEP,
		AFTER_INITIATE_READING,
		WAITING_FOR_ECP_BEFORE_EXECUTE_MOTION,
		AFTER_EXECUTE_MOTION,
		AFTER_GET_READING,
		GS_FINISHED
	};

	GEN_SET_PHASE phase; // faza w ktorej znajduje sie generator
	bool new_pulse_checked; // czy nowy puls zostal sprawdzony (wykorzystywane w scheduller_run() )
	bool wait_for_ECP_pulse; // okresla czy przed next step move ma sie zawieszac w oczekwianiu na puls z ECP 
		// wykorzystywane przy luznej i sporadycznej wspolpracy robotow.
	mp_task& mp_t;

	bool trigger; // informacja czy pszyszedl puls trigger
	
	// mapa wszystkich czujnikow
	std::map <SENSOR_ENUM, sensor*> sensor_m;
	
	// mapa wszystkich transmiterow
	std::map <TRANSMITTER_ENUM, transmitter*> transmitter_m;
	
	// mapa wszystkich robotow
	std::map <ROBOT_ENUM, mp_robot*> robot_m;
	
	mp_generator(mp_task& _mp_task);

	virtual ~mp_generator(void);
	virtual bool first_step (void) = 0;
	// generuje pierwszy krok ruchu -
	// pierwszy krok czesto rozni sie od pozostalych,
	// np. do jego generacji nie wykorzystuje sie czujnikow
	// (zadanie realizowane przez klase konkretna)
	
	virtual bool next_step (void) = 0;
	// generuje kazdy nastepny krok ruchu
	// (zadanie realizowane przez klase konkretna)

	void re_run(void); // powrot do stanu wyjsciowego

	// Kopiuje dane z robotow do generatora
	void copy_data(std::map <ROBOT_ENUM, mp_robot*>& _robot_m);
	
	// Kopiuje polecenie stworzone w generatorze do robotow
	void copy_generator_command (std::map <ROBOT_ENUM, mp_robot*>& _robot_m);

  class MP_error {  // Klasa obslugi bledow generatora na poziomie MP
    public:
      const uint64_t error_class;
      const uint64_t mp_error;
      MP_error (uint64_t err0, uint64_t err1);
  }; // end: class MP_error

}; // end: class mp_generator
// ------------------------------------------------------------------------

// ########################################################################################################
// ########################################################################################################
// ########################### GENERATORY RUCHU DLA ECP (opracowane by Jarosz) ############################
// ########################################################################################################
// ########################################################################################################

// condition to wait for desired time in ms

class mp_delay_ms_condition: public mp_generator 
{
protected:
	mp_timer* local_timer;
	float sec;
	int ms_delay;
 
public:

    // konstruktor
    mp_delay_ms_condition(mp_task& _mp_task, int _ms_delay);
    ~mp_delay_ms_condition();

	void configure (int _ms_delay);
	
	virtual bool first_step ();
	virtual bool next_step ();    

}; // end : class mp_set_next_ecps_state_generator



// generator for setting the next ecps state

class mp_set_next_ecps_state_generator : public mp_generator 
{
protected:
	int mp_2_ecp_next_state, mp_2_ecp_next_state_variant; 
	char mp_2_ecp_next_state_string[MP_2_ECP_STRING_SIZE]; // skojarzone z NEXT_STATE	
 
public:

    // konstruktor
    mp_set_next_ecps_state_generator(mp_task& _mp_task);

	void configure (int l_mp_2_ecp_next_state, int l_mp_2_ecp_next_state_variant, char* l_mp_2_ecp_next_state_string);
	
	bool first_step ();
	bool next_step ();    

}; // end : class mp_set_next_ecps_state_generator


// generator for sending end_motion mesage to ecps

class mp_send_end_motion_to_ecps_generator : public mp_generator 
{
public:

    // konstruktor
    mp_send_end_motion_to_ecps_generator(mp_task& _mp_task);
	
	bool first_step ();
	bool next_step ();    

}; // end : class mp_send_end_motion_to_ecps_state_generator



// ####################################################################################################
// Rozszerzony Generator pusty. Faktyczna generacja trajektorii odbywa sie w ECP
// ####################################################################################################

class mp_extended_empty_generator : public mp_generator {
    // Klasa dla generatorow trajektorii
    // Sluzy zarowno do wyznaczania nastepnej wartosci zadanej jak i
    // sprawdzania spelnienia warunku koncowego
 protected:
    bool activate_trigger;
    
 public:
	mp_extended_empty_generator(mp_task& _mp_task);

  ~mp_extended_empty_generator(){ };

	void configure (bool l_activate_trigger);

  bool first_step ();
      // generuje pierwszy krok ruchu -
      // pierwszy krok czesto rozni sie od pozostalych,
      // np. do jego generacji nie wykorzystuje sie czujnikow
      // (zadanie realizowane przez klase konkretna)
  bool next_step ();
     // generuje kazdy nastepny krok ruchu
     // (zadanie realizowane przez klase konkretna)

}; // end: class empty_generator


// ####################################################################################################
// Generator pusty. Faktyczna generacja trajektorii odbywa sie w ECP
// ####################################################################################################

class mp_empty_generator : public mp_generator {
    // Klasa dla generatorow trajektorii
    // Sluzy zarowno do wyznaczania nastepnej wartosci zadanej jak i
    // sprawdzania spelnienia warunku koncowego
 public:
	mp_empty_generator(mp_task& _mp_task);

  ~mp_empty_generator(){ };

  virtual bool first_step ();
      // generuje pierwszy krok ruchu -
      // pierwszy krok czesto rozni sie od pozostalych,
      // np. do jego generacji nie wykorzystuje sie czujnikow
      // (zadanie realizowane przez klase konkretna)
  virtual bool next_step ();
     // generuje kazdy nastepny krok ruchu
     // (zadanie realizowane przez klase konkretna)

}; // end: class empty_generator

// ####################################################################################################
// KLASA BAZOWA dla generatorow o zadany przyrost polozenia/orientacji
// ####################################################################################################

class mp_delta_generator : public mp_generator
{
protected:
   int node_counter;               // biezacy wezel interpolacji

public:
	mp_delta_generator(mp_task& _mp_task);
	trajectory_description irp6ot_td;
	trajectory_description irp6p_td;

}; // end : class MP_delta_generator

// ####################################################################################################
// Generator prostoliniowy o zadany przyrost polozenia/orientacji
// ####################################################################################################

class mp_tight_coop_generator : public mp_delta_generator
{

public:
  // konstruktor
	mp_tight_coop_generator(mp_task& _mp_task, trajectory_description irp6ot_tr_des, trajectory_description irp6p_tr_des);

  // destruktor
  ~mp_tight_coop_generator();

  virtual bool first_step ();

  virtual bool next_step ();

}; // end: class tight_coop_generator



// ########################################################################################################
// ####################################    KONIEC GENERATOROW   ###########################################
// ########################################################################################################



#endif
