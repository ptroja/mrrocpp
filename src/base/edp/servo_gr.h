// -------------------------------------------------------------------------
//                            servo_gr.h
// Definicje struktur danych i metod dla procesu SERVO_GROUP
//
// Ostatnia modyfikacja: 16.04.98
// -------------------------------------------------------------------------

#ifndef __SERVO_GR_H
#define __SERVO_GR_H

#include <boost/utility.hpp>
#ifndef __QNXNTO__
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition.hpp>
#endif
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"
#include "base/lib/condition_synchroniser.h"
#include "base/edp/edp_typedefs.h"

namespace mrrocpp {
namespace edp {
namespace common {

class regulator;
class HardwareInterface;
class motor_driven_effector;

const uint8_t ERROR_DETECTED = 1;
const uint8_t NO_ERROR_DETECTED = 0;

const uint64_t ALL_RIGHT = 0x0000000000000000ULL;
const uint64_t SYNCHRO_ZERO = 0x0000000000000001ULL;
const uint64_t SYNCHRO_SWITCH_ON = 0x0000000000000002ULL;
const uint64_t SYNCHRO_SWITCH_ON_AND_SYNCHRO_ZERO = 0x0000000000000003ULL;
const uint64_t LOWER_LIMIT_SWITCH = 0x0000000000000004ULL;
const uint64_t UPPER_LIMIT_SWITCH = 0x0000000000000008ULL;
const uint64_t OVER_CURRENT = 0x0000000000000010ULL;

const int SYNCHRO_NS = 10; // liczba krokow rozpedzania/hamowania
const int SYNCHRO_STOP_STEP_NUMBER = 250; // liczba krokow zatrzymania podczas synchronziacji
const int SYNCHRO_FINAL_STOP_STEP_NUMBER = 25; // liczba krokow zatrzymania podczas synchronziacji

/*-----------------------------------------------------------------------*/
class servo_buffer : public boost::noncopyable
{
	// Bufor polecen przysylanych z EDP_MASTER dla SERVO
	// Obiekt z algorytmem regulacji
private:
#ifdef __QNXNTO__
	int edp_caller; // by 7&Y
#endif

protected:
	boost::thread *thread_id;

	HardwareInterface* hi; // obiekt odpowiedzialny za kontakt ze sprzetem

	// regulator_group

	regulator* regulator_ptr[lib::MAX_SERVOS_NR];
	// tablica wskaznikow na regulatory bazowe,
	// ktore zostana zastapione regulatorami konkretnymi

	// input_buffer

	// output_buffer
	lib::servo_group_reply servo_data; // informacja przesylana do EDP_MASTER
#ifdef __QNXNTO__
	name_attach_t *attach; // 7&Y
#endif
	bool send_after_last_step; // decyduje, czy po realizacji ostatniego
	// kroku makrokroku ma byc wyslane aktualne
	// polozenie walu silnika do EDP_MASTER
	lib::edp_error reply_status; // okresla blad jaki nalezy zasygnalizowac
	// przy nastepnym kontakcie z EDP_MASTER
	lib::edp_error reply_status_tmp; // okresla blad jaki wystapil ostatnim kroku
	// pid_t caller;                    // Identyfikator EDP_MASTER

	uint8_t Move_1_step(void); // wykonac ruch o krok
	uint8_t Move_a_step(void); // wykonac ruch o krok nie reagujac na SYNCHRO_SWITCH i SYNCHRO_T
	uint8_t convert_error(void); // kompresja numeru bledu w reply_status.error0
	void reply_to_EDP_MASTER(void); // przeslanie stanu SERVO do EDP_MASTER

	void clear_reply_status(void);

	void clear_reply_status_tmp(void);

#ifdef __QNXNTO__
protected:
	int servo_fd;
public:

	int servo_to_tt_chid;
#else
	bool servo_command_rdy;
	boost::mutex servo_command_mtx;

	bool sg_reply_rdy;
	boost::mutex sg_reply_mtx;
	boost::condition sg_reply_cond;
#endif

public:
	lib::condition_synchroniser thread_started;

	lib::edp_master_command command; // polecenie z EDP_MASTER dla SERVO
	double axe_inc_per_revolution[lib::MAX_SERVOS_NR];
	double synchro_step_coarse[lib::MAX_SERVOS_NR];
	double synchro_step_fine[lib::MAX_SERVOS_NR];
	int synchro_axis_order[lib::MAX_SERVOS_NR];

	lib::edp_master_command servo_command; // polecenie z EDP_MASTER dla SERVO_GROUP
	lib::servo_group_reply sg_reply; // bufor na informacje odbierane z SERVO_GROUP

	void set_robot_model_servo_algorithm(const lib::c_buffer &instruction); // zmiana narzedzia

	void send_to_SERVO_GROUP();

	void operator()(void);

	//! input_buffer
	motor_driven_effector &master;

	lib::SERVO_COMMAND command_type(void) const;

	virtual void load_hardware_interface(void);

	virtual void get_all_positions(void);

	//! konstruktor
	servo_buffer(motor_driven_effector &_master);

	//! destruktor
	virtual ~servo_buffer(void);

	//! odczytanie polecenia z EDP_MASTER o ile zostalo przyslane
	bool get_command(void);

	//! stanie w miejscu
	void Move_passive(void);

	//! wykonac makrokrok ruchu
	void Move(void);

	//! odczytac aktualne polozenie
	void Read(void);

	//! zmienic algorytm serworegulacji lub jego parametry
	void Change_algorithm(void);

	//! synchronizacja
	virtual void synchronise(void);

	//! wybor osi
	void synchro_choose_axis_to_move(common::regulator* &crp, int j);

	//! ruch w kierunku obszaru synchronizacji az do wykrycia wylacznika synchronizacji
	int move_to_synchro_area(common::regulator* &crp, int j);

	//! wybor osi
	int synchro_stop_for_a_while(common::regulator* &crp, int j);

	//! zjazd z obszaru synchronizacji az do wykrycia wylacznika synchronizacji
	void move_from_synchro_area(common::regulator* &crp, int j);

	//! przejazd do zera enkdoera po zjezdzie z wylacznika synchronizacji
	int synchro_move_to_encoder_zero(common::regulator* &crp, int j);

	//! obliczenie nastepnej wartosci zadanej dla wszystkich napedow
	uint64_t compute_all_set_values(void);

	//! wydruk - do celow uruchomieniowych !!!
	void ppp(void) const;
};
/*-----------------------------------------------------------------------*/

} // namespace common
} // namespace edp
} // namespace mrrocpp

#endif
