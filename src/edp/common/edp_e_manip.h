// -------------------------------------------------------------------------
//                                   edp.h
// Definicje struktur danych i metod dla procesu EDP
//
// Ostatnia modyfikacja: 16.04.98
// -------------------------------------------------------------------------

#ifndef __EDP_E_MANIP_H
#define __EDP_E_MANIP_H

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"
#include "lib/srlib.h"

#include "lib/messip/messip.h"
#include "kinematics/common/transformer_error.h"
#include "kinematics/common/kinematics_manager.h"
#include "edp/common/edp_e_manip_and_conv.h"


// Konfigurator
#include "lib/configurator.h"

namespace mrrocpp {
namespace edp {
namespace common {


// base class for EDP robots with manipulators

class manip_effector: public common::manip_and_conv_effector
{

protected:
    void compute_xyz_euler_zyz (const lib::c_buffer &instruction);     // obliczenia dla ruchu ramienia (koncowka: XYZ_EULER_ZYZ)

    void compute_xyz_angle_axis (const lib::c_buffer &instruction); // obliczenia dla ruchu ramienia (koncowka: XYZ_ANGLE_AXIS)

    void compute_frame (const lib::c_buffer &instruction);             // obliczenia dla ruchu ramienia (koncowka: FRAME)

    void set_tool_frame_in_kinematic_model(const lib::Homog_matrix& hm);

    // lib::r_buffer


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

    lib::frame_tab current_end_effector_frame;
    // Podstawowa postac reprezentujaca ostatnio
    // odczytane wspolrzedne zewnetrzne koncowki
    // manipulatora wzgledem ukladu bazowego (polozenie w mm)

public:
    manip_effector (lib::configurator &_config, lib::robot_name_t l_robot_name );       // konstruktor

    void synchronise (); // synchronizacja robota
    void get_controller_state (lib::c_buffer &instruction); // synchronizacja robota

    virtual void create_threads ();

    // wyznaczenie polozenia lokalnego i globalnego transformera
    // przepisanie lokalnego zestawu lokalnego edp_servo na globalny (chronione mutexem)
    void master_joints_and_frame_download(void);// by Y przepisanie z zestawu globalnego na lokalny dla edp_master

    virtual void master_order(MT_ORDER nm_task, int nm_tryb);
};

} // namespace common
} // namespace edp
} // namespace mrrocpp

#endif
