/*
 * DataCondition.h
 *
 *  Created on: Apr 28, 2010
 *      Author: ptroja
 */

#ifndef DATACONDITION_H_
#define DATACONDITION_H_

class DataCondition {
public:
	virtual bool isNewData() const = 0;
};

#endif /* DATACONDITION_H_ */
