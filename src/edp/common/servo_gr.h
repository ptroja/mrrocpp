// -------------------------------------------------------------------------
//                            servo_gr.h
// Definicje struktur danych i metod dla procesu SERVO_GROUP
//
// Ostatnia modyfikacja: 16.04.98
// -------------------------------------------------------------------------

#ifndef __SERVO_GR_H
#define __SERVO_GR_H

#include <boost/utility.hpp>

#include "edp/common/edp.h"
#include "edp/common/hi_rydz.h"
#include "edp/common/regulator.h"

#define ERROR_DETECTED     1
#define NO_ERROR_DETECTED  0

namespace mrrocpp {
namespace edp {
namespace common {

/*-----------------------------------------------------------------------*/
class servo_buffer : public boost::noncopyable
{
    // Bufor polecen przysylanych z EDP_MASTER dla SERVO
    // Obiekt z algorytmem regulacji
private:
#ifdef __QNXNTO__
    int edp_caller;						// by 7&Y
#endif

protected:
    boost::thread *thread_id;

    hardware_interface* hi;    // obiekt odpowiedzialny za kontakt ze sprzetem

    // regulator_group

    regulator* regulator_ptr[MAX_SERVOS_NR];
    // tablica wskaznikow na regulatory bazowe,
    // ktore zostana zastapione regulatorami konkretnymi

    // input_buffer


    // output_buffer
    lib::servo_group_reply servo_data;    // informacja przesylana do EDP_MASTER
#ifdef __QNXNTO__
    name_attach_t *attach; // 7&Y
#endif
    bool send_after_last_step;    // decyduje, czy po realizacji ostatniego
    // kroku makrokroku ma byc wyslane aktualne
    // polozenie walu silnika do EDP_MASTER
    lib::edp_error reply_status;          // okresla blad jaki nalezy zasygnalizowac
    // przy nastepnym kontakcie z EDP_MASTER
    lib::edp_error reply_status_tmp;      // okresla blad jaki wystapil ostatnim kroku
    // pid_t caller;                    // Identyfikator EDP_MASTER

    uint8_t Move_1_step (void);         // wykonac ruch o krok
    uint8_t Move_a_step (void);         // wykonac ruch o krok nie reagujac na SYNCHRO_SWITCH i SYNCHRO_T
    uint8_t convert_error (void);       // kompresja numeru bledu w reply_status.error0
    void reply_to_EDP_MASTER (void); // przeslanie stanu SERVO do EDP_MASTER

    void clear_reply_status ( void );

    void clear_reply_status_tmp ( void );

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
	lib::boost_condition_synchroniser thread_started;

    lib::edp_master_command command; // polecenie z EDP_MASTER dla SERVO
	double axe_inc_per_revolution[MAX_SERVOS_NR];
	double synchro_step_coarse[MAX_SERVOS_NR];
	double synchro_step_fine[MAX_SERVOS_NR];
	int synchro_axis_order[MAX_SERVOS_NR];

    lib::edp_master_command servo_command;    // polecenie z EDP_MASTER dla SERVO_GROUP
    lib::servo_group_reply sg_reply;          // bufor na informacje odbierane z SERVO_GROUP

	void set_robot_model_servo_algorithm(lib::c_buffer &instruction); // zmiana narzedzia

    void send_to_SERVO_GROUP ();

    void operator()(void);

    //! input_buffer
    motor_driven_effector &master;

    lib::SERVO_COMMAND command_type(void) const;

    virtual void load_hardware_interface (void);

    virtual void get_all_positions (void);

    //! konstruktor
    servo_buffer (motor_driven_effector &_master);

    //! destruktor
    virtual ~servo_buffer (void);

    //! odczytanie polecenia z EDP_MASTER o ile zostalo przyslane
    bool get_command (void);

    //! stanie w miejscu
    void Move_passive (void);

    //! wykonac makrokrok ruchu
    void Move (void);

    //! odczytac aktualne polozenie
    void Read (void);

    //! zmienic algorytm serworegulacji lub jego parametry
    void Change_algorithm (void);

    //! synchronizacja
    virtual void synchronise (void);

    //! obliczenie nastepnej wartosci zadanej dla wszystkich napedow
    uint64_t compute_all_set_values (void);

    //! wydruk - do celow uruchomieniowych !!!
    void ppp (void) const;
};
/*-----------------------------------------------------------------------*/

} // namespace common
} // namespace edp
} // namespace mrrocpp

#endif
