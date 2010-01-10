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

#if defined(USE_MESSIP_SRR)
#include <messip.h>
#endif

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

	virtual void compute_frame(const lib::c_buffer &instruction); // obliczenia dla ruchu ramienia (koncowka: FRAME)

	void set_tool_frame_in_kinematic_model(const lib::Homog_matrix& hm);

	// lib::r_buffer

	lib::Homog_matrix servo_current_frame_wo_tool; // by Y dla watku EDP_SERVO    XXXXX
	lib::Homog_matrix global_current_frame_wo_tool;// globalne dla procesu EDP    XXXXXX

	lib::Homog_matrix desired_end_effector_frame; //  XXXXX
	// Podstawowa postac reprezentujaca zadane
	// wspolrzedne zewnetrzne koncowki manipulatora
	// wzgledem ukladu bazowego (polozenie w mm)

	lib::Homog_matrix current_end_effector_frame;
	// Podstawowa postac reprezentujaca ostatnio
	// odczytane wspolrzedne zewnetrzne koncowki
	// manipulatora wzgledem ukladu bazowego (polozenie w mm)

    lib::Ft_vector global_force_msr; // sila we wspolrzednych kartezjankich    XXXXX
    // 	i	 odczytana bezposrednio z czujnika - zestaw globalny dla procesu EDP

    boost::mutex force_mutex;	// mutex do sily   XXXXXX


public:
	manip_effector(lib::configurator &_config, lib::robot_name_t l_robot_name); // konstruktor

    lib::Homog_matrix return_current_frame(TRANSLATION_ENUM translation_mode);// by Y przepisanie z zestawu globalnego na lokalny edp_force

    void force_msr_upload(const lib::Ft_vector l_vector);// by Y wgranie globalnego zestawu danych

    void force_msr_download(lib::Ft_vector& l_vector);// by Y odczytanie globalnego zestawu danych


	void synchronise(); // synchronizacja robota
	void get_controller_state(lib::c_buffer &instruction); // synchronizacja robota

	virtual void set_rmodel(lib::c_buffer &instruction); // zmiana narzedzia
	virtual void get_rmodel(lib::c_buffer &instruction); // odczytanie narzedzia

	virtual void get_arm_position_get_arm_type_switch(lib::c_buffer &instruction); // odczytanie pozycji ramienia sprzetowo z sb

	void single_thread_move_arm(lib::c_buffer &instruction);
	void multi_thread_move_arm(lib::c_buffer &instruction);
	void single_thread_master_order(common::MT_ORDER nm_task, int nm_tryb);

	virtual servo_buffer* return_created_servo_buffer();

};

} // namespace common
} // namespace edp
} // namespace mrrocpp

#endif
