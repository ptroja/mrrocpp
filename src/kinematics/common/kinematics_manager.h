/*!
 * \file kinematics_manager.h
 * \brief File containing the declaration of kinematics_manager class.
 *
 * \author tkornuta
 * \date Nov 26, 2009
 */

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
 * \brief Class responsible for management of multiple kinematic models. Base class of all embodied-robot effectors.
 *
 * \author tkornuta
 * \date Nov 26, 2009
 */
class kinematics_manager
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
		//! Destroys kinematics available on the list.
		virtual ~kinematics_manager(void);

		//! Adds new kinematic model to the list.
		void add_kinematic_model(kinematic_model* _model);

		//! Sets current kinematic model.
		void set_kinematic_model(int);

		//! Returns pointer to current kinematic model.
		kinematic_model* get_current_kinematic_model(void);

		//! Returns number of given kinematic model.
		int get_current_kinematic_model_no(void);

};

} // namespace common
} // namespace kinematics
} // namespace mrrocpp

#endif

