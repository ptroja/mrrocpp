/*
 * OrDataCondition.h
 *
 *  Created on: Apr 28, 2010
 *      Author: ptroja
 */

#ifndef ORDATACONDITION_H_
#define ORDATACONDITION_H_

#include <vector>
#include <ostream>

#include "AndDataCondition.h"
#include "DataCondition.h"

class OrDataCondition : public std::vector<AndDataCondition>, public DataCondition {
	//! overloaded display operator
	friend std::ostream& operator<<(std::ostream& output, const OrDataCondition& p);

public:
	//! Base container data type
	typedef std::vector<AndDataCondition> base_t;

	//! assign a single 'Buffer' condition
	OrDataCondition & operator=(const DataBufferBase &op);

	//! assign a single 'And' condition
	//OrDataCondition & operator=(const AndDataCondition &op);

	//! compose 'Or' condition
	OrDataCondition operator|(const DataBufferBase &op);

	//! compose 'Or' condition
	OrDataCondition operator|(const AndDataCondition &op);

	//! compose 'Or' condition
	OrDataCondition operator|(const OrDataCondition &op);

	//! constructor from single 'Buffer' condition
	OrDataCondition(const DataBufferBase &op);

	//! constructor from single 'And' condition
	OrDataCondition(const AndDataCondition &op);

	//! default constructor
	OrDataCondition();

	//! check if condition is satisfied
	bool isNewData(void) const;
};

#endif /* ORDATACONDITION_H_ */
