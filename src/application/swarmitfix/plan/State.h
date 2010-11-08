/*
 * State.h
 *
 *  Created on: Jun 24, 2010
 *      Author: ptrojane
 */

#ifndef STATE_H_
#define STATE_H_

class State {
protected:
	//! index of situation, when the move towards this position should start
	int TindFrom;

	//! index of situation, when the move should end and the required position should be reached
	int TindTo;
};

#endif /* STATE_H_ */
