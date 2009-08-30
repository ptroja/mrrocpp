// -------------------------------------------------------------------------
//                                   edp.h
// Definicje struktur danych i metod dla procesu EDP
//
// Ostatnia modyfikacja: 16.04.98
// -------------------------------------------------------------------------

#ifndef __EDP_E_MANIP_AND_CONV_H
#define __EDP_E_MANIP_AND_CONV_H

#include <stdint.h>
#include <pthread.h>

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"
#include "lib/srlib.h"

#include "messip/messip.h"
#include "kinematics/common/transformer_error.h"
#include "kinematics/common/kinematics_manager.h"
#include "edp/common/edp_effector.h"
// Konfigurator
#include "lib/configurator.h"


namespace mrrocpp {
namespace edp {
namespace common {



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

    int servo_fd;

    // numer serwo chwytaka
    short gripper_servo_nr;

    pthread_t serwo_tid;
    pthread_t reader_tid;
    pthread_t trans_t_tid;
    pthread_t vis_t_tid;

    STATE next_state;    // stan nastepny, do ktorego przejdzie EDP_MASTER

    friend class servo_buffer;

    lib::edp_master_command servo_command;    // polecenie z EDP_MASTER dla SERVO_GROUP
    lib::servo_group_reply sg_reply;          // bufor na informacje odbierane z SERVO_GROUP

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

#ifdef __QNXNTO__
    int servo_to_tt_chid;
#else
    sem_t servo_command_ready, sg_reply_ready;
#endif

    virtual void initialize (void) = 0;

    in_out_buffer in_out_obj; // bufor wejsc wyjsc
    reader_buffer rb_obj;
    master_trans_t_buffer mt_tt_obj;

    manip_and_conv_effector (lib::configurator &_config, lib::ROBOT_ENUM l_robot_name);       // konstruktor
    virtual ~manip_and_conv_effector();

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
    // przygotowuje odpowiedz dla ECP

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
    bool pre_synchro_motion(lib::c_buffer &instruction) const;


    // Czy robot zsynchronizowany? // by Y - wziete z ecp
    bool is_synchronised ( void ) const;

};
/************************ edp_irp6s_and_conv_effector ****************************/




} // namespace common
} // namespace edp
} // namespace mrrocpp

#endif
