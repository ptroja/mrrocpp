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

#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"

#include <boost/thread/thread.hpp>

namespace mrrocpp {
namespace edp {
namespace common {
class motor_driven_effector;

// Struktura z informacja, ktore elementy struktury reader_data maja byc zapisane do pliku
struct reader_config
{
	reader_config();

	bool step; // numer kroku

	bool measure_time; // czas wykonania pomiaru (w ms)

	bool desired_inc[lib::MAX_SERVOS_NR]; // wejscie dla osi 0,2,3,4
	bool current_inc[lib::MAX_SERVOS_NR]; // wyjscie

	bool pwm[lib::MAX_SERVOS_NR]; // wypelnienie PWM
	bool uchyb[lib::MAX_SERVOS_NR]; // wypelnienie PWM
	bool abs_pos[lib::MAX_SERVOS_NR];

	bool force[6]; // pierwsze 3 z 6
	bool desired_force[6]; // pierwsze 3 z 6
	bool filtered_force[6]; // sila po przefiltrowaniu

	bool current_joints[lib::MAX_SERVOS_NR];
	bool measured_current[lib::MAX_SERVOS_NR];

	bool desired_cartesian_position[6]; // skaldowe liniowe polozenia zadanego
	bool real_cartesian_position[6]; // polozenie rzeczywiste
	bool real_cartesian_vel[6]; // predkosc rzeczywista
	bool real_cartesian_acc[6]; // przyspieszenie rzeczywiste
	bool servo_mode; // by Y 0 - petla bierna 1- wykonywanie zleconego przemieszczenia
};

struct reader_data
{ // Struktura z danymi pomiarowymi w reader do zapisu do pliku
	unsigned long step; // numer kroku
	struct timespec measure_time; // czas wykonania pomiaru (w ms)

	float desired_inc[lib::MAX_SERVOS_NR]; // wejscie dla osi 0,2,3,4
	short int current_inc[lib::MAX_SERVOS_NR]; // wyjscie
	// float current_position[6];
	float pwm[lib::MAX_SERVOS_NR]; // wypelnienie PWM
	float uchyb[lib::MAX_SERVOS_NR]; // wypelnienie PWM
	double abs_pos[lib::MAX_SERVOS_NR];

	double force[3]; // pierwsze 3 z 6
	double desired_force[3]; // pierwsze 3 z 6
	double filtered_force[6]; // sila po przefiltrowaniu

	double desired_cartesian_position[6]; // skaldowe liniowe polozenia zadanego
	double current_joints[lib::MAX_SERVOS_NR]; // spolozenie w joints
	int measured_current[lib::MAX_SERVOS_NR]; // prad w zalozeniu w [ma]

	double real_cartesian_position[6]; // polozenie rzeczywiste
	double real_cartesian_vel[6]; // predkosc rzeczywista
	double real_cartesian_acc[6]; // przyspieszenie rzeczywiste
	bool servo_mode; // by Y: false - petla bierna, true - wykonywanie zleconego przemieszczenia
	bool ui_trigger; // by Y: false - nie wystapil w biezacym kroku, true - wystapil
};

/**************************** reader_buffer *****************************/

class reader_buffer : public boost::noncopyable
{
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
	motor_driven_effector &master;

	boost::thread thread_id;

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
