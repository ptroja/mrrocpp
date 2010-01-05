//// ------------------------------------------------------------------------
// Proces:		EDP
// Plik:			kinematics_manager.cc
// System:	QNX/MRROC++  v. 6.3
// Opis:		Klasa zarzadzajaca modelami kinematyki. 
//				- deklaracja klasy
//				- wspolna dla wszystkich robotow
//
// Autor:		tkornuta
// Data:		17.01.2007
// ------------------------------------------------------------------------

#ifndef __EDP_KIN_MAN_H
#define __EDP_KIN_MAN_H

#include <map>
#include "kinematics/common/kinematic_model.h"

using namespace std;

namespace mrrocpp {
namespace kinematics {
namespace common {

/*!
 * \class kinematic_manager
 * \brief Class responsible for
 * basic six kinematic methods: direct, inverse, i2e, e2i, mp2i and i2mp.
 *
 * \author tkornuta
 * \date Nov 26, 2009
 */
class kinematic_manager
{
	protected:
		//! List of available kinematic models for given effector.
		map<short, kinematic_model*> kinematic_models_list;

		//! Number of currently selected model.
		int current_kinematic_model_no;

		//! Pointer to currently selected kinematic model.
		kinematic_model* current_kinematic_model;

		//! Abstract method, implemented in the effector class - creates a list of available kinematic models for given effector.
		virtual void create_kinematic_models_for_given_robot(void) = 0;

	public:
		// Destruktor - niszczy liste kinematyk.
		virtual ~kinematic_manager(void);

		// Dodaje nowy kinematic_model_with_tool na koniec listy.
		void add_kinematic_model(kinematic_model* _model);

		// Ustawia kinematic_model_with_tool kinematyki.
		void set_kinematic_model(int);
		// Zwraca obecny kinematic_model_with_tool kinematyki.
		kinematic_model* get_current_kinematic_model(void);

		// Zwraca number obecnie wybranego modelu kinematyki.
		int get_current_kinematic_model_no(void);

};// manager

} // namespace common
} // namespace kinematics
} // namespace mrrocpp

#endif

