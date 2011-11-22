/*!
 * @file
 * @brief File containing the definition of kinematics_manager methods.
 *
 * @author tkornuta
 * @date Nov 26, 2009
 *
 * @ingroup KINEMATICS
 */

#include "base/lib/com_buf.h"
#include "base/kinematics/kinematics_manager.h"

#include "base/lib/exception.h"

using namespace mrrocpp::lib::exception;

namespace mrrocpp {
namespace kinematics {
namespace common {

kinematics_manager::kinematics_manager()
	: current_kinematic_model_no(-1)
{
}

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

void kinematics_manager::set_kinematic_model(unsigned int _desired_kinematic_model_nr)
{
	if (_desired_kinematic_model_nr >= kinematic_models_list.size()){

		BOOST_THROW_EXCEPTION(nfe_2() << mrrocpp_error0(INVALID_KINEMATIC_MODEL_NO));
	}

	current_kinematic_model_no = _desired_kinematic_model_nr;
}

void kinematics_manager::add_kinematic_model(kinematic_model* _model)
{
	// Add new model to the end of the list.
	kinematic_models_list[kinematic_models_list.size()] = _model;
}


int kinematics_manager::get_current_kinematic_model_no(void)
{
	return current_kinematic_model_no;
}

} // namespace common
} // namespace kinematics
} // namespace mrrocpp
