// -------------------------------------------------------------------------
//                            servo_gr.h
// Definicje struktur danych i metod dla procesu SERVO_GROUP
//
// Ostatnia modyfikacja: 16.04.98
// -------------------------------------------------------------------------

#ifndef __SERVO_GR_H
#define __SERVO_GR_H

#include "edp/common/edp.h"
#include "edp/common/hi_rydz.h"
#include "edp/common/edp_extension_thread.h"

#define ERROR_DETECTED     1
#define NO_ERROR_DETECTED  0

namespace mrrocpp {
namespace edp {
namespace common {

class regulator;


/*-----------------------------------------------------------------------*/
class servo_buffer  : public edp_extension_thread
{
    // Bufor polecen przysylanych z EDP_MASTER dla SERVO
    // Obiekt z algorytmem regulacji
private:
#ifdef __QNXNTO__
    int edp_caller;						// by 7&Y
#endif

protected:

    hardware_interface* hi;    // obiekt odpowiedzialny za kontakt ze sprzetem

    // regulator_group

    regulator* regulator_ptr[MAX_SERVOS_NR];
    // tablica wskaznikow na regulatory bazowe,
    // ktore zostana zastapione regulatorami konkretnymi


    // input_buffer
    lib::edp_master_command command; // polecenie z EDP_MASTER dla SERVO

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
    virtual uint8_t Move_a_step (void);         // wykonac ruch o krok nie reagujac na SYNCHRO_SWITCH i SYNCHRO_T
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
    lib::edp_master_command servo_command;    // polecenie z EDP_MASTER dla SERVO_GROUP
    lib::servo_group_reply sg_reply;          // bufor na informacje odbierane z SERVO_GROUP
    void send_to_SERVO_GROUP ();
    static void *thread_start(void* arg);
    void *thread_main_loop(void* arg);

    void create_thread(void);

    manip_and_conv_effector &master;
    // input_buffer
    lib::SERVO_COMMAND command_type(void);
    // by Yoyek & 7 -  typ returna na lib::SERVO_COMMAND
    virtual void load_hardware_interface (void);
    // output_buffer
    virtual void get_all_positions (void);
    //servo_buffer ();             // konstruktor
    servo_buffer (manip_and_conv_effector &_master);             // konstruktor
    virtual ~servo_buffer (void);      // destruktor
    bool get_command (void);      // odczytanie polecenia z EDP_MASTER
    // o ile zostalo przyslane
    void Move_passive (void);        // stanie w miejscu
    void Move (void);                // wykonac makrokrok ruchu
    void Read (void);                // odczytac aktualne polozenie
    void Change_algorithm (void);    // zmienic algorytm serworegulacji lub jego parametry
    virtual void synchronise (void);         // synchronizacja
    virtual uint64_t compute_all_set_values (void);
    // obliczenie nastepnej wartosci zadanej dla wszystkich napedow

    void ppp (void) const;                 // wydruk - do celow uruchomieniowych !!!
};
/*-----------------------------------------------------------------------*/

} // namespace common
} // namespace edp
} // namespace mrrocpp

#endif
