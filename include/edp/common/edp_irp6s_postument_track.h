// -------------------------------------------------------------------------
//                                   edp.h
// Definicje struktur danych i metod dla procesu EDP
//
// Ostatnia modyfikacja: 16.04.98
// -------------------------------------------------------------------------

#ifndef __EDP_IRP6S_P_T_H
#define __EDP_IRP6S_P_T_H



#include "edp/common/edp.h"
#include "edp/common/edp_force_sensor.h"



// base class for EDP robots with manipulators postument and on_track


/************************ edp_irp6s_effector ****************************/
class edp_irp6s_postument_track_effector: public edp_irp6s_effector
{

private:
    // informacja czy czujnik sily zostal skonfigurowany po synchronizacji robota
    bool force_sensor_configured;

protected:

    // czy chwytak wlaczony ?
    short is_gripper_active;

    pthread_t force_tid;
    pthread_t edp_vsp_tid;


    void arm_abs_xyz_eul_zyz_2_frame (double *p);
    // Przeksztalcenie definicji koncowki z postaci
    // XYZ_EULER_ZYZ wyrazonej bezwzglednie do postaci
    // FRAME oraz przepisanie wyniku przeksztalcenia do
    // wewnetrznych struktur danych TRANSFORMATORa

    void arm_frame_2_xyz_eul_zyz ();

    // Przeksztalcenie definicji koncowki z postaci
    // FRAME do postaci XYZ_EULER_ZYZ
    // oraz przepisanie wyniku przeksztalcenia do
    // wewnetrznych struktur danych REPLY_BUFFER
    void arm_frame_2_pose_force_torque_at_frame (bool* = NULL);
    // Przeksztalcenie definicji koncowki z postaci
    // FRAME do postaci XYZ_EULER_ZYZ
    // oraz przepisanie wyniku przeksztalcenia do
    // wewnetrznych struktur danych REPLY_BUFFER

    // r_buffer

    double global_kartez_force_msr[7], prevoius_global_kartez_force_msr[7]; // sila we wspolrzednych kartezjankich    XXXXX
    // 	i	 odczytana bezposrednio z czujnika - zestaw globalny dla procesu EDP

    pthread_mutex_t force_mutex;	// mutex do sily   XXXXXX

public:

    void set_rmodel (c_buffer *instruction);                    // zmiana narzedzia
    void get_rmodel (c_buffer *instruction);                    // odczytanie narzedzia

    sr_vsp *sr_msg;		//!< komunikacja z SR
    sem_t new_ms; //!< semafor dostepu do nowej wiadomosci dla vsp
    edp_force_sensor *vs;
    bool TERMINATE;			//!< zakonczenie obydwu watkow

    static void *edp_vsp_thread_start(void* arg);
    void *edp_vsp_thread(void* arg);
    static void *force_thread_start(void* arg);
    void *force_thread(void* arg);


    bool force_sensor_do_configure; // FLAGA ZLECENIA KONFIGURACJI CZUJNIKA
    virtual void initialize (void);
    edp_irp6s_postument_track_effector (configurator &_config, ROBOT_ENUM l_robot_name );       // konstruktor

    int force_tryb;

    void move_arm (c_buffer *instruction);            // przemieszczenie ramienia

    void get_arm_position (bool read_hardware, c_buffer *instruction); // odczytanie pozycji ramienia

    // ruch pozycyjno silowo dla staqlej orientacji i kierukow liniowych
    virtual void pose_force_linear_move (c_buffer *instruction);

    void pose_force_torque_at_frameove (c_buffer *instruction);

    void servo_joints_and_frame_actualization_and_upload(void);// by Y

    void force_msr_upload(double *new_value);// by Y wgranie globalnego zestawu danych

    // value to 7 elemetnowa tablica short
    // tryb - 0 zestaw kartezjanski, 1 zestaw nieprzetworzony

    Homog_matrix return_current_frame(TRANSLATION_ENUM translation_mode);// by Y przepisanie z zestawu globalnego na lokalny edp_force

    // wyznaczenie polozenia lokalnego i globalnego transformera
    // przepisanie lokalnego zestawu lokalnego edp_servo na globalny (chronione mutexem)
    void master_joints_and_frame_download(void);// by Y przepisanie z zestawu globalnego na lokalny dla edp_master

    frame_tab force_current_end_effector_frame;// by Y dla watku EDP_FORCE

    void force_msr_download(double *new_value, double *old_value);// by Y odczytanie globalnego zestawu danych

    void create_threads ();

};
/************************ edp_irp6s_effector ****************************/




#endif
