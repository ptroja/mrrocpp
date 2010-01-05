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
#include "kinematics/common/simple_kinematic_model.h"

namespace mrrocpp {
namespace kinematics {
namespace common {

class manager
{
	protected:
		// Lista kinematyk.
		std::map<short, simple_kinematic_model*> kinematic_models_list;
		// Number obecnie wybranego modelu kinematyki.
		int current_kinematic_model_no;
		// Obecnie wybrany kinematic_model_with_tool kinematyki.
		simple_kinematic_model* current_kinematic_model;

		// Tworzy liste modeli kinematyki - metoda abstrakcyjna, implementowana w klasie ROBOT (!).
		virtual void create_kinematic_models_for_given_robot(void) = 0;

	public:
		// Konstruktor - tworzy liste kinematyk.
		manager(void);
		// Destruktor - niszczy liste kinematyk.
		virtual ~manager(void);

		// Dodaje nowy kinematic_model_with_tool na koniec listy.
		void add_kinematic_model(simple_kinematic_model* _model);

		// Ustawia kinematic_model_with_tool kinematyki.
		void set_kinematic_model(int);
		// Zwraca obecny kinematic_model_with_tool kinematyki.
		simple_kinematic_model* get_current_kinematic_model(void);

		// Zwraca number obecnie wybranego modelu kinematyki.
		int get_current_kinematic_model_no(void);

};// manager

} // namespace common
} // namespace kinematics
} // namespace mrrocpp

#endif

