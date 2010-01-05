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

#include "lib/com_buf.h"
#include "kinematics/common/kinematics_manager.h"

#include "edp/common/exception.h"
using namespace mrrocpp::edp::common::exception;

namespace mrrocpp {
namespace kinematics {
namespace common {


manager::manager(void)
{
};//: manager



manager::~manager(void)
{
    // tutaj usuniecie kinematyk z listy oraz samej listy.
};//:~manager


void manager::set_kinematic_model (int _desired_kinematic_model_nr)
{
    if (_desired_kinematic_model_nr >= kinematic_models_list.size() || _desired_kinematic_model_nr <0 )
    {
        throw NonFatal_error_2 (INVALID_KINEMATIC_MODEL_NO);
    }
    current_kinematic_model = (simple_kinematic_model*) (kinematic_models_list[_desired_kinematic_model_nr]);
    current_kinematic_model_no = _desired_kinematic_model_nr;
    // Wypisanie nazwy modelu kinematyki.
  //  master->msg->message(current_kinematic_model->get_kinematic_model_label());
}


void manager::add_kinematic_model (simple_kinematic_model* _model)
{
    // Dodanie nowego modelu na koniec listy.
    kinematic_models_list[kinematic_models_list.size()] = _model;
}

simple_kinematic_model* manager::get_current_kinematic_model (void)
{
    return current_kinematic_model;
}

int manager::get_current_kinematic_model_no (void)
{
    return current_kinematic_model_no;
}

} // namespace common
} // namespace kinematics
} // namespace mrrocpp
