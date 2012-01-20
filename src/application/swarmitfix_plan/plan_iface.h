/*
 * plan_iface.cc
 *
 *  Created on: Jan 20, 2012
 *      Author: ptroja
 */

#ifndef PLAN_IFACE_H_
#define PLAN_IFACE_H_

#include <memory>

// Forward declaration.
class Plan;

//! Read and validate plan from file.
::std::auto_ptr< ::Plan > readPlanFromFile(const std::string & path);

#endif /* PLAN_IFACE_H_ */
