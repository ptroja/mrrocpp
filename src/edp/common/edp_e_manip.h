/*!
 * \file edp_e_manip.h
 * \brief File containing the declaration of edp::common::motor_driven_effector class.
 *
 * \author yoyek
 * \date 2009
 *
 */

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
#include "edp/common/edp_e_motor_driven.h"

// Konfigurator
#include "lib/configurator.h"

namespace mrrocpp {
namespace edp {
namespace common {

//
/*!
 * \brief base class for EDP robotic manipulators
 *
 * It extends motor_driven_effector by the methods of computation of the end--effector position with direct and inverse kinematic task
 * and some methods of govering data for the purpose of position--force control.
 */
class manip_effector: public common::motor_driven_effector
{

protected:

	/*!
	 * \brief
	 *
	 *
	 */
	virtual void compute_frame(const lib::c_buffer &instruction); // obliczenia dla ruchu ramienia (koncowka: FRAME)

	/*!
	 * \brief
	 *
	 *
	 */
	lib::Homog_matrix servo_current_frame_wo_tool; // by Y dla watku EDP_SERVO    XXXXX

	/*!
	 * \brief
	 *
	 *
	 */
	lib::Homog_matrix global_current_frame_wo_tool;// globalne dla procesu EDP    XXXXXX

	/*!
	 * \brief
	 *
	 *
	 */
	lib::Homog_matrix desired_end_effector_frame; //  XXXXX
	// Podstawowa postac reprezentujaca zadane
	// wspolrzedne zewnetrzne koncowki manipulatora
	// wzgledem ukladu bazowego (polozenie w mm)

	/*!
	 * \brief
	 *
	 *
	 */
	lib::Homog_matrix current_end_effector_frame;
	// Podstawowa postac reprezentujaca ostatnio
	// odczytane wspolrzedne zewnetrzne koncowki
	// manipulatora wzgledem ukladu bazowego (polozenie w mm)

	/*!
	 * \brief
	 *
	 *
	 */
    lib::Ft_vector global_force_msr; // sila we wspolrzednych kartezjankich    XXXXX
    // 	i	 odczytana bezposrednio z czujnika - zestaw globalny dla procesu EDP

	/*!
	 * \brief
	 *
	 *
	 */
    boost::mutex force_mutex;	// mutex do sily   XXXXXX

	/*!
	 * \brief
	 *
	 *
	 */
	void single_thread_move_arm(lib::c_buffer &instruction);

	/*!
	 * \brief
	 *
	 *
	 */
	void multi_thread_move_arm(lib::c_buffer &instruction);


public:

	/*!
	 * \brief
	 *
	 *
	 */
	manip_effector(lib::configurator &_config, lib::robot_name_t l_robot_name); // konstruktor

	/*!
	 * \brief
	 *
	 *
	 */
    lib::Homog_matrix return_current_frame(TRANSLATION_ENUM translation_mode);// by Y przepisanie z zestawu globalnego na lokalny edp_force

	/*!
	 * \brief
	 *
	 *
	 */
    void force_msr_upload(const lib::Ft_vector l_vector);// by Y wgranie globalnego zestawu danych

	/*!
	 * \brief
	 *
	 *
	 */
    void force_msr_download(lib::Ft_vector& l_vector);// by Y odczytanie globalnego zestawu danych

	/*!
	 * \brief
	 *
	 *
	 */
	virtual bool servo_joints_and_frame_actualization_and_upload(void); // by Y

	/*!
	 * \brief
	 *
	 *
	 */
	void synchronise(); // synchronizacja robota

	/*!
	 * \brief
	 *
	 *
	 */
	void get_controller_state(lib::c_buffer &instruction); // synchronizacja robota

	/*!
	 * \brief
	 *
	 *
	 */
	virtual void set_rmodel(lib::c_buffer &instruction); // zmiana narzedzia

	/*!
	 * \brief
	 *
	 *
	 */
	virtual void get_rmodel(lib::c_buffer &instruction); // odczytanie narzedzia


	/*!
	 * \brief
	 *
	 *
	 */
	virtual void get_arm_position_get_arm_type_switch(lib::c_buffer &instruction); // odczytanie pozycji ramienia sprzetowo z sb
};

} // namespace common
} // namespace edp
} // namespace mrrocpp

#endif
