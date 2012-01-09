/*
 * WorkersStatus.cc
 *
 *  Created on: Dec 11, 2011
 *      Author: ptroja
 */

#include <boost/foreach.hpp>

#include "WorkersStatus.h"

#include "robot/spkm/const_spkm1.h"
#include "robot/spkm/const_spkm2.h"
#include "robot/smb/const_smb1.h"
#include "robot/smb/const_smb2.h"
#include "robot/shead/const_shead1.h"
#include "robot/shead/const_shead2.h"
#include "robot/sbench/const_sbench.h"

WorkersStatus::WorkersStatus()
{
	status[lib::spkm1::ROBOT_NAME] = IDLE;
	status[lib::spkm2::ROBOT_NAME] = IDLE;
	status[lib::smb1::ROBOT_NAME] = IDLE;
	status[lib::smb2::ROBOT_NAME] = IDLE;
	status[lib::shead1::ROBOT_NAME] = IDLE;
	status[lib::shead2::ROBOT_NAME] = IDLE;
	status[lib::sbench::ROBOT_NAME] = IDLE;
}
#include <iostream>
bool WorkersStatus::allIdle() const
{
	std::cout << "WorkersStatus::allIdle()" << std::endl;
	BOOST_FOREACH(const container_t::value_type & v, status) {
		if (v.second == BUSY) return false;
	}

	return true;
}

WorkersStatus::container_t::mapped_type & WorkersStatus::operator[](const container_t::key_type & k)
{
	container_t::iterator it = status.find(k);
	if(it != status.end())
		return it->second;

	throw std::runtime_error("Access status of non-existing worker");
}
