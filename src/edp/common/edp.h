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


} // namespace common
} // namespace edp
} // namespace mrrocpp

#endif
