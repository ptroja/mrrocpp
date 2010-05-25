// -------------------------------------------------------------------------
//                                   reader.h
// -------------------------------------------------------------------------

#ifndef __READER_H
#define __READER_H

#include <stdint.h>

#include <boost/utility.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition_variable.hpp>
#include <boost/circular_buffer.hpp>

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"
#include "lib/mis_fun.h"

#include <boost/thread/thread.hpp>

class motor_driven_effector;

namespace mrrocpp {
namespace edp {
namespace common {

// Struktura z informacja, ktore elementy struktury reader_data maja byc zapisane do pliku
struct reader_config
{
	bool step;       // numer kroku

    bool msec; // czas wykonania pomiaru (w ms)

    bool desired_inc[MAX_SERVOS_NR];       // wejscie dla osi 0,2,3,4
    bool current_inc[MAX_SERVOS_NR]; // wyjscie

    bool pwm[MAX_SERVOS_NR];       // wypelnienie PWM
    bool uchyb[MAX_SERVOS_NR];                 // wypelnienie PWM
    bool abs_pos[MAX_SERVOS_NR];

    bool force[6]; // pierwsze 3 z 6
    bool desired_force[6]; // pierwsze 3 z 6
    bool filtered_force[6]; // sila po przefiltrowaniu

    bool current_joints[MAX_SERVOS_NR];

    bool desired_cartesian_position[6]; // skaldowe liniowe polozenia zadanego
    bool real_cartesian_position[6]; // polozenie rzeczywiste
    bool real_cartesian_vel[6]; // predkosc rzeczywista
    bool real_cartesian_acc[6]; // przyspieszenie rzeczywiste
    bool servo_mode; // by Y 0 - petla bierna 1- wykonywanie zleconego przemieszczenia
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

    double desired_cartesian_position[6]; // skaldowe liniowe polozenia zadanego
    double current_joints[MAX_SERVOS_NR]; // spolozenie w joints

    double real_cartesian_position[6]; // polozenie rzeczywiste
    double real_cartesian_vel[6]; // predkosc rzeczywista
    double real_cartesian_acc[6]; // przyspieszenie rzeczywiste
    bool servo_mode; // by Y: false - petla bierna, true - wykonywanie zleconego przemieszczenia
    bool ui_trigger; // by Y: false - nie wystapil w biezacym kroku, true - wystapil
};

/**************************** reader_buffer *****************************/

class reader_buffer : public boost::noncopyable
{
private:
    motor_driven_effector &master;

    boost::thread *thread_id;
public:
	boost::mutex reader_mutex;

	boost::condition_variable cond;

	bool new_data;

	//! main thread loop
    void operator()();

    reader_data step_data; // dane pomiarowe dla biezacego mikrokroku
    reader_config reader_cnf; //   Struktura z informacja, ktore elementy struktury reader_data maja byc zapisane do pliku

    reader_buffer(motor_driven_effector &_master);
    ~reader_buffer();


};
/**************************** end of reader_buffer *****************************/



} // namespace common
} // namespace edp
} // namespace mrrocpp

#endif
