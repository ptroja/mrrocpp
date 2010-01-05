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


kinematic_manager::~kinematic_manager(void)
{
	// Delete list of kinematics.
	while (!kinematic_models_list.empty()) {
		// Free memory from model and erase list element.
		kinematic_model* km = (kinematic_model*) (kinematic_models_list.begin()->second);
		delete(km);
		kinematic_models_list.erase( kinematic_models_list.begin());
	}//: while

};//:~manager


void kinematic_manager::set_kinematic_model (int _desired_kinematic_model_nr)
{
    if (_desired_kinematic_model_nr >= kinematic_models_list.size() || _desired_kinematic_model_nr <0 )
    {
        throw NonFatal_error_2 (INVALID_KINEMATIC_MODEL_NO);
    }
    current_kinematic_model = (kinematic_model*) (kinematic_models_list[_desired_kinematic_model_nr]);
    current_kinematic_model_no = _desired_kinematic_model_nr;
}


void kinematic_manager::add_kinematic_model (kinematic_model* _model)
{
    // Dodanie nowego modelu na koniec listy.
    kinematic_models_list[kinematic_models_list.size()] = _model;
}

kinematic_model* kinematic_manager::get_current_kinematic_model (void)
{
    return current_kinematic_model;
}

int kinematic_manager::get_current_kinematic_model_no (void)
{
    return current_kinematic_model_no;
}

} // namespace common
} // namespace kinematics
} // namespace mrrocpp
