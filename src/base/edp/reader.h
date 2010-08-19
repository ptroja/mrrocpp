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

#include <ctime>

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"

#include <boost/thread/thread.hpp>

#include "lib/mrmath/ft_v_vector.h"

class motor_driven_effector;

namespace mrrocpp {
namespace edp {
namespace common {

// Struktura z informacja, ktore elementy struktury reader_data maja byc zapisane do pliku
struct reader_config
{
	reader_config();

	bool step;       // numer kroku

    bool measure_time; // czas wykonania pomiaru (w ms)

    bool desired_inc[MAX_SERVOS_NR];       // wejscie dla osi 0,2,3,4
    bool current_inc[MAX_SERVOS_NR]; // wyjscie

    bool pwm[MAX_SERVOS_NR];       // wypelnienie PWM
    bool uchyb[MAX_SERVOS_NR];                 // wypelnienie PWM
    bool abs_pos[MAX_SERVOS_NR];

    bool force[6]; // pierwsze 3 z 6

    bool current_joints[MAX_SERVOS_NR];

    bool desired_cartesian_position[6]; // skaldowe liniowe polozenia zadanego
    bool real_cartesian_position[6]; // polozenie rzeczywiste
    bool servo_mode; // by Y 0 - petla bierna 1- wykonywanie zleconego przemieszczenia
};

struct reader_data
{   // Struktura z danymi pomiarowymi w reader do zapisu do pliku
    unsigned long step;       // numer kroku
    struct timespec measure_time; // czas wykonania pomiaru (w ms)

    float desired_inc[MAX_SERVOS_NR];       // wejscie dla osi 0,2,3,4
    short int current_inc[MAX_SERVOS_NR]; // wyjscie

    float pwm[MAX_SERVOS_NR];       // wypelnienie PWM
    float uchyb[MAX_SERVOS_NR];                 // wypelnienie PWM
    double abs_pos[MAX_SERVOS_NR];

    lib::Ft_vector force;

    lib::Xyz_Euler_Zyz_vector desired_cartesian_position; // skaldowe liniowe polozenia zadanego
    double current_joints[MAX_SERVOS_NR]; // spolozenie w joints

    lib::Xyz_Euler_Zyz_vector real_cartesian_position; // polozenie rzeczywiste

    bool servo_mode; // by Y: false - petla bierna, true - wykonywanie zleconego przemieszczenia
    bool ui_trigger; // by Y: false - nie wystapil w biezacym kroku, true - wystapil
};

/**************************** reader_buffer *****************************/

class reader_buffer : public boost::noncopyable
{
private:
    motor_driven_effector &master;

    boost::thread thread_id;
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
private:
    bool write_csv;

    void write_header_old_format(std::ofstream& outfile);
    void write_data_old_format(std::ofstream& outfile, const reader_data & data);

    void write_header_csv(std::ofstream& outfile);
    void write_data_csv(std::ofstream& outfile, const reader_data & data);
};
/**************************** end of reader_buffer *****************************/



} // namespace common
} // namespace edp
} // namespace mrrocpp

#endif
