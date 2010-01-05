/*!
 * \file kinematics_manager.cc
 * \brief File containing the definition of kinematics_manager methods.
 *
 * \author tkornuta
 * \date Nov 26, 2009
 */

#include "lib/com_buf.h"
#include "kinematics/common/kinematics_manager.h"

#include "edp/common/exception.h"
using namespace mrrocpp::edp::common::exception;

namespace mrrocpp {
namespace kinematics {
namespace common {

kinematics_manager::~kinematics_manager(void)
{
	// Delete list of kinematics.
	while (!kinematic_models_list.empty()) {
		// Free memory from model and erase list element.
		kinematic_model* km = (kinematic_model*) (kinematic_models_list.begin()->second);
		delete (km);
		kinematic_models_list.erase(kinematic_models_list.begin());
	}//: while
}

void kinematics_manager::set_kinematic_model(int _desired_kinematic_model_nr)
{
	if (_desired_kinematic_model_nr >= kinematic_models_list.size() || _desired_kinematic_model_nr < 0)
		throw NonFatal_error_2(INVALID_KINEMATIC_MODEL_NO);

	current_kinematic_model = (kinematic_model*) (kinematic_models_list[_desired_kinematic_model_nr]);
	current_kinematic_model_no = _desired_kinematic_model_nr;
}

void kinematics_manager::add_kinematic_model(kinematic_model* _model)
{
	// Dodanie nowego modelu na koniec listy.
	kinematic_models_list[kinematic_models_list.size()] = _model;
}

kinematic_model* kinematics_manager::get_current_kinematic_model(void)
{
	return current_kinematic_model;
}

int kinematics_manager::get_current_kinematic_model_no(void)
{
	return current_kinematic_model_no;
}

} // namespace common
} // namespace kinematics
} // namespace mrrocpp
