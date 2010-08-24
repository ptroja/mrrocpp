/*
 * AndDataCondition.h
 *
 *  Created on: Apr 28, 2010
 *      Author: ptroja
 */

#ifndef ANDDATACONDITION_H_
#define ANDDATACONDITION_H_

#include <vector>
#include <ostream>

#include "DataBufferBase.h"
#include "DataCondition.h"
#include "OrDataCondition.h"

//! representation of 'And' buffer condition
class AndDataCondition : public std::vector<const DataBufferBase *>, public DataCondition {
	//! overloaded display operator
	friend std::ostream& operator<<(std::ostream& output, const AndDataCondition& p);

public:
	//! assign a single 'Buffer' condition
	AndDataCondition & operator=(const DataBufferBase &op);

	//! compose 'And' condition
	AndDataCondition operator&(const DataBufferBase &op);

	//! compose 'And' condition
	AndDataCondition operator&(const AndDataCondition &op);

	//! compose 'Or' condition
	OrDataCondition operator|(const AndDataCondition &op);

	//! compose 'Or' condition
	OrDataCondition operator|(const OrDataCondition &op);

	//! constructor from single 'Buffer' condition
	explicit AndDataCondition(const DataBufferBase &op);

	//! default constructor
	AndDataCondition();

	//! check if this condition was satisfied
	bool isNewData() const;
};

#endif /* ANDDATACONDITION_H_ */
