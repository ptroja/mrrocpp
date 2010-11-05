/*
 * DataBufferBase.h
 *
 *  Created on: Apr 28, 2010
 *      Author: ptroja
 */

#ifndef DATABUFFERBASE_H_
#define DATABUFFERBASE_H_

#include <string>

#include "base/lib/xdr/xdr_iarchive.hpp"

// forward declarations
class AndDataCondition;
class OrDataCondition;

class DataBufferBase {
	//! Agent needs an access to Store/Update methods
	friend class Agent;

protected:
	//! name of the data buffer
	const std::string name;

	//! flag for marking a new data
	bool new_data_ready;

	//! store new data
	virtual void Store(xdr_iarchive<> & ia) = 0;

	//! update buffer if new data has arrived
	virtual void Update(void) = 0;

public:
	//! Constructor
	DataBufferBase(const std::string & _name);

	//! compose 'And' condition
	AndDataCondition operator&(DataBufferBase &op);

	//! compose 'And' condition
	AndDataCondition operator&(AndDataCondition &op);

	//! compose 'Or' condition
	OrDataCondition operator|(DataBufferBase &op);

	//! compose 'Or' condition
	OrDataCondition operator|(AndDataCondition &op);

	//! compose 'Or' condition
	OrDataCondition operator|(OrDataCondition &op);

	//! get name of the buffer
	const std::string & getName() const;

	//! This is required to make a class polimorphic
	virtual ~DataBufferBase();

	//! check if condition is satisfied
	bool isNewData(void) const;
};

#endif /* DATABUFFERBASE_H_ */
