#if !defined(_ECP_ROBOT_H)
#define _ECP_ROBOT_H

#include "common/com_buf.h"
#include "lib/srlib.h"
#include "lib/configurator.h"

#include "messip/messip.h"

// ------------------------------------------------------------------------
class ecp_buffer: public ecp_command_buffer, public edp_reply_buffer {
	public:
		sr_ecp* sr_ecp_msg;     // by Y - Wskaznik na obiekt do komunikacji z SR
#if !defined(USE_MESSIP_SRR)
		void send  (int);
		void query (int);
#else
		void send  (messip_channel_t *);
		void query (messip_channel_t *);
#endif
};
// ------------------------------------------------------------------------

struct robot_transmission_data {

	robot_transmission_data (void)
		: instruction_type(INVALID), reply_type(ACKNOWLEDGE)
	{};
	
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
};

struct robot_EDP_transmission_data : robot_transmission_data {

	// by Y - do sily

	double ECPtoEDP_inertia[6], ECPtoEDP_reciprocal_damping[6];
	double ECPtoEDP_position_velocity[MAX_SERVOS_NR], ECPtoEDP_force_xyz_torque_xyz[6];
	BEHAVIOUR_SPECIFICATION ECPtoEDP_behaviour[6];
	//	bool selection_vector[6];

	// r_buffer
	frame_tab  current_beggining_arm_frame;      // trojscian koncowki wzgledem ukladu bazowego
	frame_tab  current_predicted_arm_frame;      // trojscian koncowki wzgledem ukladu bazowego
	frame_tab  current_present_arm_frame;      // trojscian koncowki wzgledem ukladu bazowego
	// double pos_xyz_rot_xyz[6];
	double EDPtoECP_force_xyz_torque_xyz[6];

	// end by Y

	// moment zadany dla Dunga
	double desired_torque[MAX_SERVOS_NR];
};

class ecp_task;

class ecp_robot {

		// Klasa bazowa dla robotow (klasa abstrakcyjna)
		// Kazdy robot konkretny (wyprowadzony z klasy bazowej)
		// musi zawierac pola danych (skladowe) dotyczace
		// ostatnio zrealizowanej pozycji oraz pozycji zadanej
	
		// int fd;	// by Y&W
	protected:

		// int UI_fd; // by Y&W - przeniesione z procesu master
		pid_t EDP_MASTER_Pid; // Identyfikator procesu driver'a edp_m

		// strukture EDP_command_and_reply_buffer.instruction, ktora jest
		// nastepnie wyslana przez funkcje execute_motion() do EDP.
		// Struktura EDP_command_and_reply_buffer.reply_package zawierajaca
		// odpowiedz EDP na wyslany rozkaz, ktora moze byc wykorzystana
		// przez generator.next_step()
		// Funkcja generator.next_step() przygotowuje rozkazy dla EDP wypelniajac

		// by Y&W - przerzucenie zainicjowania komunikacji z procesu master do klasy
		void connect_to_edp (const char* edp_net_attach_point);

	public:
		bool synchronised; // Flaga synchronizacji robota (true - zsynchronizowany, false - nie)

		int number_of_servos;

		ecp_buffer EDP_command_and_reply_buffer;
#if !defined(USE_MESSIP_SRR)
		int EDP_fd;	// by Y&W
#else
		messip_channel_t *EDP_fd;
#endif
		const ROBOT_ENUM robot_name; // by Y - nazwa robota (track, postument etc.)
		bool communicate; // okresla czy robot ma byc obslugiwany w Move

		robot_EDP_transmission_data EDP_data; // Obraz robota wykorzystywany przez generator

		virtual void execute_motion (void);
		// Zlecenie wykonania ruchu przez robota (realizowane przez klase konkretna):
		// na poziomie ECP jest to polecenie dla EDP
		
		ecp_robot(ROBOT_ENUM _robot_name, configurator &_config, sr_ecp *_sr_ecp);
		ecp_robot(ROBOT_ENUM _robot_name, ecp_task& _ecp_object);

		pid_t get_EDP_pid(void) const;

		// destruktor by Y - do usuniecia obiektu do komunikacji z SR
		virtual ~ecp_robot(void);

		virtual void synchronise ( void );
		// Zlecenie synchronizacji robota
		// Pobranie aktualnych polozen

		virtual void create_command (void) = 0;
		// wypelnia bufor wysylkowy do EDP na podstawie danych zawartych w obrazie
		// robota wykorzystywanych przez generator
		// Ten bufor znajduje sie w robocie

		virtual void get_reply (void)  = 0;
		// pobiera z pakietu przeslanego z EDP informacje i wstawia je do
		// odpowiednich skladowych obrazu robota wykorzystywanych przez generator
		// Ten bufor znajduje sie w robocie

		// Kopiowanie bufora przesylanego z MP do bufora wysylanego do EDP
		virtual void copy_mp_to_edp_buffer (c_buffer& mp_buffer);

		// by Y - o dziwo tego nie bylo !!!
		// Kopiowanie bufora przesylanego z EDP do bufora wysylanego do MP
		virtual void copy_edp_to_mp_buffer (r_buffer& mp_buffer);

		bool is_synchronised ( void ) const; // Czy robot zsynchronizowany?

		class ECP_error {  // Klasa obslugi bledow robota
			public:
				uint64_t error_class;
				uint64_t error_no;
				edp_error error;

				ECP_error ( uint64_t err_cl, uint64_t err_no, uint64_t err0 = 0, uint64_t err1 = 0);
		}; // end: class ECP_error

		class ECP_main_error {  // Klasa obslugi bledow ECP
			public:
				uint64_t error_class;
				uint64_t error_no;

				ECP_main_error ( uint64_t err_cl, uint64_t err_no);
		}; // end: class ECP_main_error // by Y&W przerzucone do wnetrza klasy

};

#endif /* _ECP_ROBOT_H */
