// ------------------------------------------------------------------------
// Proces:		EDP
// Plik:			kinematics_manager.cc
// System:	QNX/MRROC++  v. 6.3
// Opis:		Klasa zarzadzajaca modelami kinematyki.
//				- definicja metod klasy
//				- wspolna dla wszystkich robotow
//
// Autor:		tkornuta
// Data:		19.01.2007
// ------------------------------------------------------------------------

#include "common/typedefs.h"
#include "common/impconst.h"
#include "common/com_buf.h"
#include "edp/common/edp.h"

#include "edp/common/kinematics_manager.h"
#include "edp/common/kinematic_model.h"

// extern edp_effector* master;   // Bufor polecen i odpowiedzi EDP_MASTER

// Konstruktor - tworzy liste kinematyk.
kinematics_manager::kinematics_manager(void)
{
    //	create_kinematics_list_for_given_robot();
}
;//: kinematics_manager


// Destruktor - niszczy liste kinematyk.
kinematics_manager::~kinematics_manager(void)
{
    // tutaj usuniecie kinematyk z listy oraz samej listy.
}
;//:~kinematics_manager

// Zmiana aktywnego modelu kinematyki.
void kinematics_manager::set_kinematic_model (int _desired_kinematic_model_nr)
{
    if (_desired_kinematic_model_nr >= kinematic_models_list.size() || _desired_kinematic_model_nr <0 )
    {
        throw transformer_error::NonFatal_error_2 (INVALID_KINEMATIC_MODEL_NO);
    };
    current_kinematic_model = (kinematic_model*) (kinematic_models_list[_desired_kinematic_model_nr]);
    current_kinematic_model_no = _desired_kinematic_model_nr;
    // Wypisanie nazwy modelu kinematyki.
  //  master->msg->message(current_kinematic_model->get_kinematic_model_label());
}
;//: set_kinematic_model


// Zmiana aktywnego modelu kinematyki.
void kinematics_manager::add_kinematic_model (kinematic_model* _model)
{
    // Dodanie nowego modelu na koniec listy.
    kinematic_models_list[kinematic_models_list.size()] = _model;
}
;//: add_kinematic_model

// Zwraca obecnie wybrany modelu kinematyki.
kinematic_model* kinematics_manager::get_current_kinematic_model (void)
{
    return current_kinematic_model;
};

// Zwraca number obecnie wybranego modelu kinematyki.
int kinematics_manager::get_current_kinematic_model_no (void)
{
    return current_kinematic_model_no;
}
