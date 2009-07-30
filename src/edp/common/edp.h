// -------------------------------------------------------------------------
//                                   edp.h
// Definicje struktur danych i metod dla procesu EDP
//
// Ostatnia modyfikacja: 16.04.98
// -------------------------------------------------------------------------

#ifndef __EDP_H
#define __EDP_H

#include <stdint.h>
#include <semaphore.h>
#include <pthread.h>
#include <sys/dispatch.h>

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"
#include "lib/srlib.h"

#include "messip/messip.h"
#include "kinematics/common/transformer_error.h"
#include "kinematics/common/kinematics_manager.h"

// Konfigurator
#include "lib/configurator.h"


namespace mrrocpp {
namespace edp {
namespace common {

class reader_buffer;
class master_trans_t_buffer;
class in_out_buffer;

class irp6s_postument_track_effector;

enum STATE { GET_STATE, GET_SYNCHRO, SYNCHRO_TERMINATED, GET_INSTRUCTION, EXECUTE_INSTRUCTION, WAIT, WAIT_Q };
// extern int errno;

enum TRANSLATION_ENUM { WITH_TRANSLATION, WITHOUT_TRANSLATION };
// extern int errno;

/*--------------------------------------------------------------------*/


// Glowna klasa efektora EDP
class effector : public kinematic::common::transformer_error
{
protected:

    // faktyczny typ odpowiedzi dla ECP
    // (przechowuje typ odpowiedzi, gdy reply_type jest chwilowo zmienione)
    lib::REPLY_TYPE real_reply_type;

    // bufor odpowiedzi wysylanych do ECP/MP
    lib::r_buffer reply;

    int caller;				// by 7&Y

public:
    lib::configurator &config;
    lib::sr_edp *msg;

    void check_config(const char* string, uint8_t* input);
    bool initialize_communication (void);

    virtual void initialize (void) = 0;
    std::string mrrocpp_network_path;

#if !defined(USE_MESSIP_SRR)

    name_attach_t *attach;
#else /* USE_MESSIP_SRR */

    messip_channel_t *attach;
#endif /* USE_MESSIP_SRR */

    effector (lib::configurator &_config, lib::ROBOT_ENUM l_robot_name);       // konstruktor
    lib::controller_state_t controller_state_edp_buf; // do okreslenia stanu robota

    int test_mode;

    // oczekuje na polecenie od ECP, wczytuje je,
    // okresla typ nadeslanej instrukcji
    lib::INSTRUCTION_TYPE receive_instruction (void); // by YW

    // wyslanie adekwatnej odpowiedzi do ECP
    void reply_to_instruction (void);

    void insert_reply_type (lib::REPLY_TYPE rt);

    virtual void main_loop(); // main loop
    virtual void create_threads () = 0;

    bool is_reply_type_ERROR() const;

    void establish_error (uint64_t err0, uint64_t err1);

    lib::REPLY_TYPE is_reply_type (void) const;

    uint64_t is_error_no_0 (void) const;
    uint64_t is_error_no_1 (void) const;

    // bufory:
    // - polecen przysylanych z ECP
    // - polecen przysylanych z ECP dla watku trans_t
    lib::ecp_command_buffer new_ecp_command;
    lib::c_buffer new_instruction, current_instruction;

    const lib::ROBOT_ENUM robot_name;

    lib::POSE_SPECIFICATION previous_set_arm_type; // by Y poprzedni sposob zadawania pozycji
};
/************************ EDP_EFFECTOR ****************************/



// base class for EDP robots with manipulators and conveyor

/************************ edp_irp6s_and_conv_effector ****************************/
class manip_and_conv_effector : public effector, public kinematic::common::manager
{

protected:
    static void *reader_thread_start(void* arg);
    void *reader_thread(void* arg);
    static void *trans_thread_start(void* arg);
    void *trans_thread(void* arg);
    static void *servo_thread_start(void* arg);
    void *servo_thread(void* arg);
    static void *visualisation_thread_start(void* arg);
    void *visualisation_thread(void* arg);

    lib::WORD motion_steps;            // liczba krokow ruchu zadanego (makrokroku)

    //Liczba krokow pierwszej fazy ruchu, czyli krok, w ktorym ma zostac
    //przekazana informacja o realizacji pierwszej fazy ruchu:
    //0 < value_in_step_no <= motion_steps + 1
    //Dla value_in_step_no = motion_steps wiadomosc dotrze po zrealizowaniu
    //makrokroku, ale informacja o polozeniu bedzie dotyczyc realizacji
    //przedostatniego kroku makrokroku.
    //Dla value_in_step_no = motion_steps + 1 wiadomosc dotrze po zrealizowaniu
    //jednego kroku obiegu petli ruchu jalowego po zakonczeniu makrokroku,
    //ale informacja o polozeniu bedzie dotyczyc realizacji calego makrokroku.
    //Dla value_in_step_no < motion_steps wiadomosc dotrze przed zrealizowaniem
    //makrokroku i informacja o polozeniu bedzie dotyczyc realizacji srodkowej
    //fazy makrokroku.
    lib::WORD value_in_step_no;

    int serwo_fd;

    // numer serwo chwytaka
    short gripper_servo_nr;

    pthread_t edp_tid;
    pthread_t serwo_tid;
    pthread_t reader_tid;
    pthread_t trans_t_tid;
    pthread_t vis_t_tid;
    STATE next_state;    // stan nastepny, do ktorego przejdzie EDP_MASTER

    lib::edp_master_command servo_command;    // Polecenie z EDP_MASTER dla SERVO_GROUP

    lib::servo_group_reply sg_reply;            // bufor na informacje przesylane z SERVO_GROUP

    void set_outputs (const lib::c_buffer &instruction);                // ustawienie wyjsc binarnych

    void get_inputs (lib::r_buffer *local_reply);                 // odczytanie wejsc binarnych

    // kasuje zmienne - uwaga najpierw nalezy ustawic number_of_servos
    void reset_variables ();

    void compute_motors (const lib::c_buffer &instruction);             // obliczenia dla ruchu ramienia (silnikami)

    void compute_joints (const lib::c_buffer &instruction);             // obliczenia dla ruchu ramienia (stawami)

    void move_servos ();

    void common_synchronise();
    void common_get_controller_state(lib::c_buffer &instruction); // by Y

    // Wyslanie polecenia ruchu do SERVO_GROUP oraz odebranie wyniku
    // realizacji pierwszej fazy ruchu

    void send_to_SERVO_GROUP ();
    // Wyslanie polecenia do procesu SERVO_GROUP i odebranie odpowiedzi



    void arm_joints_2_joints (void);
    // Przepisanie definicji koncowki danej w postaci
    // JOINTS z wewnetrznych struktur danych TRANSFORMATORa
    // do wewnetrznych struktur danych REPLY_BUFFER

    void arm_motors_2_motors (void);
    // Przepisanie definicji koncowki danej w postaci
    // MOTORS z wewnetrznych struktur danych TRANSFORMATORa
    // do wewnetrznych struktur danych REPLY_BUFFER



    // transformer

    double servo_current_motor_pos[MAX_SERVOS_NR];   // Polozenia walow silnikow -// dla watku edp_servo    XXXX

    double global_current_motor_pos[MAX_SERVOS_NR];   // Polozenia walow silnikow -// globalne dla procesu EDP  XXXX

    double global_current_joints[MAX_SERVOS_NR];       // Wspolrzedne wewnetrzne -// globalne dla procesu EDP   XXXXX

    double servo_current_joints[MAX_SERVOS_NR];       // Wspolrzedne wewnetrzne -// dla watku EDP_SERVO   XXXXXX


    bool trans_t_command; // by Y dalej do komunikacji master z servo   XXXXX

    pthread_mutex_t edp_irp6s_effector_mutex;	// mutex    XXXXXX

    double desired_joints[MAX_SERVOS_NR];       // Wspolrzedne wewnetrzne -
    // ostatnio obliczone (zadane) (w radianach)

    double desired_joints_tmp[MAX_SERVOS_NR];       // Wspolrzedne wewnetrzne -
    // ostatnio obliczone (zadane) (w radianach) przed sprawdzeniem na ograniczenia kinematyczne

    double current_joints[MAX_SERVOS_NR];       // Wspolrzedne wewnetrzne -
    // ostatnio odczytane (w radianach) // by Y dla watku EDP_MASTER

    double previous_joints[MAX_SERVOS_NR];       // Wspolrzedne wewnetrzne -
    // obliczone (zadane) w poprzednim kroku (w radianach)

    double desired_motor_pos_old[MAX_SERVOS_NR];
    // Polozenia walow silnikow -
    // poprzednio obliczone (zadane) (w radianach)
    double desired_motor_pos_new[MAX_SERVOS_NR];
    // Polozenia walow silnikow -
    // aktualnie obliczone (zadane) (w radianach)

    double desired_motor_pos_new_tmp[MAX_SERVOS_NR];
    // Polozenia walow silnikow -
    // aktualnie obliczone (zadane) (w radianach)    przed sprawdzeniem na ograniczenia kinematyczne

    double current_motor_pos[MAX_SERVOS_NR];   // Polozenia walow silnikow -
    // ostatnio odczytane (w radianach)
    double motor_pos_increment_reading[MAX_SERVOS_NR];
    // ostatnio odczytany przyrost polozenia
    // walow silnikow (w radianach)

    int16_t PWM_value[MAX_SERVOS_NR];             // wartosci zadane wypelnienia PWM
    int16_t current[MAX_SERVOS_NR];                // prad sterujacy
    lib::MOTION_TYPE motion_type;        // sposob zadania ruchu: ABSOLUTE/RELATIVE




    lib::BYTE servo_algorithm_ecp[MAX_SERVOS_NR];
    // Tablica numerow algorytmow serworegulacji przyslanych z ECP
    lib::BYTE servo_parameters_ecp[MAX_SERVOS_NR];
    // Tablica numerow zestawow parametrow
    // algorytmow serworegulacji przyslanych z ECP
    lib::BYTE servo_algorithm_sg[MAX_SERVOS_NR];
    // Tablica numerow algorytmow serworegulacji przyslanych z SERVO_GROUP
    lib::BYTE servo_parameters_sg[MAX_SERVOS_NR];
    // Tablica numerow zestawow parametrow
    // algorytmow serworegulacji przyslanych z SERVO_GROUP
    bool synchronised;         // Flaga zsynchronizowania robota


    // stan regulatora chwytaka
    short servo_gripper_reg_state;

public:

    void master_joints_read (double*);
    int servo_to_tt_chid;
    virtual void initialize (void) = 0;

    in_out_buffer* in_out_obj; // bufor wejsc wyjsc
    reader_buffer *rb_obj;
    master_trans_t_buffer *mt_tt_obj;
    manip_and_conv_effector (lib::configurator &_config, lib::ROBOT_ENUM l_robot_name);       // konstruktor

    virtual void set_rmodel (lib::c_buffer &instruction) = 0;                    // zmiana narzedzia
    virtual void get_rmodel (lib::c_buffer &instruction) = 0;                    // odczytanie narzedzia

    unsigned long step_counter;

    short number_of_servos; // by Y ilosc serwomechanizmow  XXX
    // w zaleznosci od tego czy chwytak ma byc aktywny czy nie

    virtual void move_arm (lib::c_buffer &instruction) = 0;            // przemieszczenie ramienia

    virtual void get_arm_position (bool read_hardware, lib::c_buffer &instruction) = 0; // odczytanie pozycji ramienia

    virtual void synchronise (); // synchronizacja robota
    virtual void servo_joints_and_frame_actualization_and_upload(void) = 0; // by Y

    void main_loop(); // main loop

    virtual void create_threads ();

    void interpret_instruction (lib::c_buffer &instruction);
    // interpretuje otrzymana z ECP instrukcje;
    // wypelnaia struktury danych TRANSFORMATORa;
    // przygotowuje odpowied��� dla ECP

    // odczytanie numerow algorytmow i numerow zestawow ich parametrow
    void get_algorithms ();

    virtual void get_controller_state(lib::c_buffer &instruction); // by Y

    bool is_power_on() const;

    void update_servo_current_motor_pos(double motor_position_increment, int i);
    void update_servo_current_motor_pos_abs(double abs_motor_position, int i);

    // ustalenie formatu odpowiedzi
    lib::REPLY_TYPE rep_type (lib::c_buffer &instruction);

    // sprawdzenie czy jest to dopuszczalny rozkaz ruchu
    // przed wykonaniem synchronizacji robota
    bool pre_synchro_motion(lib::c_buffer &instruction);


    // Czy robot zsynchronizowany? // by Y - wziete z ecp
    bool is_synchronised ( void ) const;

};
/************************ edp_irp6s_and_conv_effector ****************************/



// base class for EDP robots with manipulators


/************************ edp_irp6s_effector ****************************/
class manip_effector: public common::manip_and_conv_effector
{

protected:
    void compute_xyz_euler_zyz (const lib::c_buffer &instruction);     // obliczenia dla ruchu ramienia (koncowka: XYZ_EULER_ZYZ)

    void compute_xyz_angle_axis (const lib::c_buffer &instruction); // obliczenia dla ruchu ramienia (koncowka: XYZ_ANGLE_AXIS)

    void compute_frame (const lib::c_buffer &instruction);             // obliczenia dla ruchu ramienia (koncowka: FRAME)

    void set_tool_frame_in_kinematic_model(const lib::Homog_matrix& hm);

    // lib::r_buffer

    void tool_frame_2_xyz_aa (void);
    // Przeksztalcenie definicji narzedzia z postaci
    // TOOL_FRAME do postaci TOOL_XYZ_ANGLE_AXIS oraz przepisanie wyniku
    // przeksztalcenia do wewnetrznych struktur danych
    // REPLY_BUFFER

    ////////////////////////////K

    void tool_axially_symmetrical_frame_2_xyz_eul_zy (void);

    //////////////////////////K

    void tool_frame_2_xyz_eul_zyz (void);
    // Przeksztalcenie definicji narzedzia z postaci
    // TOOL_FRAME do postaci TOOL_XYZ_EULER_ZYZ oraz przepisanie wyniku
    // przeksztalcenia do wewnetrznych struktur danych
    // REPLY_BUFFER
    void tool_frame_2_frame_rep (void);
    // Przepisanie definicji narzedzia danej w postaci TOOL_FRAME
    // z wewnetrznych struktur danych TRANSFORMATORa
    // do wewnetrznych struktur danych REPLY_BUFFER
    void arm_frame_2_xyz_aa (void);
    // Przeksztalcenie definicji koncowki z postaci
    // FRAME do postaci XYZ_ANGLE_AXIS
    // oraz przepisanie wyniku przeksztalcenia do
    // wewnetrznych struktur danych REPLY_BUFFER
    virtual void arm_frame_2_xyz_eul_zyz () = 0;


    void arm_frame_2_frame (void);
    // Przepisanie definicji koncowki danej w postaci
    // FRAME z wewnetrznych struktur danych TRANSFORMATORa
    // do wewnetrznych struktur danych REPLY_BUFFER

    // lib::c_buffer


    void tool_xyz_aa_2_frame (lib::c_buffer &instruction);
    // Przeksztalcenie definicji narzedzia z postaci
    // TOOL_XYZ_ANGLE_AXIS do postaci TOOL_FRAME oraz przepisanie wyniku
    // przeksztalcenia do wewnetrznych struktur danych
    // TRANSFORMATORa


    ////////////////////////////K

    void tool_axially_symmetrical_xyz_eul_zy_2_frame(lib::c_buffer *instruction);


    ///////////////////////////K
    void tool_xyz_eul_zyz_2_frame (lib::c_buffer &instruction);
    // Przeksztalcenie definicji narzedzia z postaci
    // TOOL_XYZ_EULER_ZYZ do postaci TOOL_FRAME oraz przepisanie wyniku
    // przeksztalcenia do wewnetrznych struktur danych
    // TRANSFORMATORa
    void tool_frame_2_frame (lib::c_buffer &instruction);
    // Przepisanie definicji narzedzia danej w postaci TOOL_FRAME
    // do wewnetrznych struktur danych TRANSFORMATORa
    void arm_abs_xyz_aa_2_frame (const double *p);
    // Przeksztalcenie definicji koncowki z postaci
    // XYZ_ANGLE_AXIS wyrazonej bezwzglednie do postaci
    // FRAME oraz przepisanie wyniku przeksztalcenia do
    // wewnetrznych struktur danych TRANSFORMATORa
    virtual void arm_abs_xyz_eul_zyz_2_frame (const double *p) = 0;
    // Przeksztalcenie definicji koncowki z postaci
    // XYZ_EULER_ZYZ wyrazonej bezwzglednie do postaci
    // FRAME oraz przepisanie wyniku przeksztalcenia do
    // wewnetrznych struktur danych TRANSFORMATORa
    void arm_abs_frame_2_frame (lib::frame_tab p_m);
    // Przepisanie definicji koncowki danej
    // w postaci FRAME wyrazonej bezwzglednie
    // do wewnetrznych struktur danych TRANSFORMATORa
    void arm_rel_xyz_aa_2_frame (const double*);
    // Przeksztalcenie definicji koncowki z postaci
    // XYZ_ANGLE_AXIS wyrazonej wzglednie do postaci
    // FRAME oraz przepisanie wyniku przeksztalcenia do
    // wewnetrznych struktur danych TRANSFORMATORa
    void arm_rel_xyz_eul_zyz_2_frame (const double*);
    // Przeksztalcenie definicji koncowki z postaci
    // XYZ_EULER_ZYZ wyrazonej wzglednie do postaci
    // FRAME oraz przepisanie wyniku przeksztalcenia do
    // wewnetrznych struktur danych TRANSFORMATORa
    void arm_rel_frame_2_frame (lib::frame_tab p_m);
    // Przepisanie definicji koncowki danej
    // w postaci FRAME wyrazonej wzglednie
    // do wewnetrznych struktur danych TRANSFORMATORa



    lib::frame_tab servo_current_frame_wo_tool; // by Y dla watku EDP_SERVO    XXXXX
    lib::Homog_matrix servo_current_end_effector_frame_with_tool_and_base; // by Y dla watku EDP_SERVO    XXXXX
    lib::Homog_matrix servo_previous_end_effector_frame_with_tool_and_base; // by Y dla watku EDP_SERVO    XXXXX

    lib::frame_tab global_current_frame_wo_tool;// globalne dla procesu EDP    XXXXXX

    // dla potrzeb wyznaczenia sztywnosci ukladu mnaipulator - drugi manipulator badz ramie czlowieka badz srodowisko
    lib::K_vector servo_xyz_angle_axis_translation;
    lib::K_vector servo_xyz_angle_axis_rotation;

    double servo_real_kartez_pos[6]; // by Y polozenie we wspolrzednych xyz_euler_zyz obliczane co krok servo   XXXXX
    double servo_real_kartez_vel[6]; // by Y predkosc we wspolrzednych xyz_euler_zyz obliczane co krok servo   XXXXX
    double servo_real_kartez_acc[6]; // by Y predkosc we wspolrzednych xyz_euler_zyz obliczane co krok servo   XXXXX


    lib::frame_tab desired_end_effector_frame;      //  XXXXX
    // Podstawowa postac reprezentujaca zadane
    // wspolrzedne zewnetrzne koncowki manipulatora
    // wzgledem ukladu bazowego (polozenie w mm)


    lib::frame_tab desired_end_effector_frame_tmp_m;      //  XXXXX
    // Podstawowa postac reprezentujaca zadane
    // wspolrzedne zewnetrzne koncowki manipulatora
    // wzgledem ukladu bazowego (polozenie w mm)       przed sprawdzeniem na ograniczenia kinematyczne

    lib::frame_tab current_end_effector_frame;
    // Podstawowa postac reprezentujaca ostatnio
    // odczytane wspolrzedne zewnetrzne koncowki
    // manipulatora wzgledem ukladu bazowego (polozenie w mm)

public:
    manip_effector (lib::configurator &_config, lib::ROBOT_ENUM l_robot_name );       // konstruktor

    void synchronise (); // synchronizacja robota
    void get_controller_state (lib::c_buffer &instruction); // synchronizacja robota

    // wyznaczenie polozenia lokalnego i globalnego transformera
    // przepisanie lokalnego zestawu lokalnego edp_servo na globalny (chronione mutexem)
    void master_joints_and_frame_download(void);// by Y przepisanie z zestawu globalnego na lokalny dla edp_master

};
/************************ edp_irp6s_effector ****************************/





class System_error
{
    // Klasa bledow systemowych zawiazanych z komunikacja miedzyprocesowa

};



// Struktura z informacja, ktore elementy struktury reader_data maja byc zapisane do pliku
struct reader_config
{
    uint8_t step;       // numer kroku

    uint8_t msec; // czas wykonania pomiaru (w ms)

    uint8_t desired_inc[MAX_SERVOS_NR];       // wejscie dla osi 0,2,3,4
    uint8_t current_inc[MAX_SERVOS_NR]; // wyjscie
    // float current_position[6];
    uint8_t pwm[MAX_SERVOS_NR];       // wypelnienie PWM
    uint8_t uchyb[MAX_SERVOS_NR];                 // wypelnienie PWM
    uint8_t abs_pos[MAX_SERVOS_NR];

    uint8_t force[6]; // pierwsze 3 z 6
    uint8_t desired_force[6]; // pierwsze 3 z 6
    uint8_t filtered_force[6]; // sila po przefiltrowaniu

    uint8_t current_joints[MAX_SERVOS_NR];

    uint8_t current_cartesian_position[6]; // skaldowe liniowe polozenia zadanego
    uint8_t real_cartesian_position[6]; // polozenie rzeczywiste
    uint8_t real_cartesian_vel[6]; // predkosc rzeczywista
    uint8_t real_cartesian_acc[6]; // przyspieszenie rzeczywiste
    uint8_t servo_mode; // by Y 0 - petla bierna 1- wykonywanie zleconego przemieszczenia

};


struct reader_data
{   // Struktura z danymi pomiarowymi w reader do zapisu do pliku
    unsigned long step;       // numer kroku
    unsigned int msec; // czas wykonania pomiaru (w ms)

    float desired_inc[MAX_SERVOS_NR];       // wejscie dla osi 0,2,3,4
    short int current_inc[MAX_SERVOS_NR]; // wyjscie
    // float current_position[6];
    float pwm[MAX_SERVOS_NR];       // wypelnienie PWM
    float uchyb[MAX_SERVOS_NR];                 // wypelnienie PWM
    double abs_pos[MAX_SERVOS_NR];

    double force[3]; // pierwsze 3 z 6
    double desired_force[3]; // pierwsze 3 z 6
    double filtered_force[6]; // sila po przefiltrowaniu

    double current_cartesian_position[6]; // skaldowe liniowe polozenia zadanego
    double current_joints[MAX_SERVOS_NR]; // spolozenie w joints

    double real_cartesian_position[6]; // polozenie rzeczywiste
    double real_cartesian_vel[6]; // predkosc rzeczywista
    double real_cartesian_acc[6]; // przyspieszenie rzeczywiste
    bool servo_mode; // by Y: false - petla bierna, true - wykonywanie zleconego przemieszczenia
    bool ui_trigger; // by Y: false - nie wystapil w biezacym kroku, true - wystapil
};



/**************************** reader_buffer *****************************/

class reader_buffer
{
private:
    sem_t reader_sem;
    pthread_mutex_t reader_mutex;

public:
    reader_data step_data; // dane pomiarowe dla biezacego mikrokroku
    reader_config reader_cnf; //   Struktura z informacja, ktore elementy struktury reader_data maja byc zapisane do pliku

    reader_buffer();

    int	set_new_step(); // podniesienie semafora
    int	reader_wait_for_new_step(); // oczekiwanie na semafor

    int	lock_mutex(); // zajecie mutex'a
    int	unlock_mutex(); // zwolnienie mutex'a

};


/**************************** end of reader_buffer *****************************/


/**************************** master_trans_t_buffer *****************************/

enum MT_ORDER { MT_GET_CONTROLLER_STATE, MT_SET_RMODEL, MT_GET_ARM_POSITION, MT_GET_ALGORITHMS, MT_MOVE_ARM, MT_SYNCHRONISE};
enum ERROR_TYPE { NO_ERROR, Fatal_erroR, NonFatal_erroR_1, NonFatal_erroR_2, NonFatal_erroR_3, NonFatal_erroR_4, System_erroR};


class master_trans_t_buffer : public kinematic::common::transformer_error
{
private:
    sem_t master_to_trans_t_sem; // semafor pomiedzy edp_master a edp_trans
    sem_t trans_t_to_master_sem; // semafor pomiedzy edp_master a edp_trans

public:

    MT_ORDER trans_t_task;
    int trans_t_tryb;
    ERROR_TYPE error;

    // wskaznik na bledy (rzutowany na odpowiedni blad)
    void* error_pointer;

    master_trans_t_buffer();

    int	master_to_trans_t_order(MT_ORDER nm_task, int nm_tryb);
    int	master_wait_for_trans_t_order_status();
    int	trans_t_to_master_order_status_ready();
    int	trans_t_wait_for_master_order();

};
/**************************** master_trans_t_buffer *****************************/


/**************************** IN_OUT_BUFFER *****************************/
class in_out_buffer
{
private:
    lib::WORD binary_input;		// wejscie binarne
    lib::BYTE analog_input[8];		// wejscie analogowe - dla 8 kanalow

    lib::WORD binary_output;		// wyjscie binarne
    intrspin_t output_spinlock; // spinlock (semafor) do wyjscia
    intrspin_t input_spinlock; // spinlock (semafor) do wejscia

public:

    // konstruktor
    in_out_buffer();

    lib::BYTE set_output_flag; // flaga czy ustawic wyjcie na robota
    void set_output (const lib::WORD *out_value);
    void get_output (lib::WORD *out_value);
    void set_input (const lib::WORD *binary_in_value, const lib::BYTE *analog_in_table);
    void get_input (lib::WORD *binary_in_value, lib::BYTE *analog_in_table);
};
/**************************** IN_OUT_BUFFER *****************************/


// Zwrocenie stworzonego obiektu - efektora. Funkcja implementowana w plikach efektorow konkretnych (jadro).
effector* return_created_efector (lib::configurator &_config);

} // namespace common
} // namespace edp
} // namespace mrrocpp

#endif
