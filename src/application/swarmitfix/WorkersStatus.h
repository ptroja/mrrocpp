/*
 * WorkersStatus.h
 *
 *  Created on: Dec 11, 2011
 *      Author: ptroja
 */

#ifndef WORKERSSTATUS_H_
#define WORKERSSTATUS_H_

#include <boost/unordered_map.hpp>

#include "base/lib/impconst.h"

using namespace mrrocpp;

class container_t;

//! Association container with status of worker agents
class WorkersStatus {
public:
	//! Type for status of single worker
	typedef enum _status { IDLE, BUSY } status_t;

private:
	//! Internal container type
	typedef boost::unordered_map<const lib::robot_name_t, status_t> container_t;

public:
	//! Initialize status of swarm agents to IDLE
	WorkersStatus();

	//! Check if all workers are idle
	bool allIdle() const;

	//! Access operator
	container_t::mapped_type & operator[](const container_t::key_type & k);

private:
	//! Internal associative container
	container_t status;
};

#endif /* WORKERSSTATUS_H_ */
