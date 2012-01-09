/*
 * PlanIterators.h
 *
 *  Created on: Jan 9, 2012
 *      Author: ptroja
 */

#ifndef PLANITERATORS_H_
#define PLANITERATORS_H_

#include "plan.hxx"

template<typename ItemSequence>
class PlanItemConstIterator : private ItemSequence::const_iterator {
private:
	//! Underlying operator
	typedef typename ItemSequence::const_iterator BaseType;

	//! Plan over which to iterate
	const ItemSequence & items;

	//! Agent id
	const int agent_id;

public:
	//! Constructor
	PlanItemConstIterator(const ItemSequence & _items, int _agent_id)
		: items(_items), agent_id(_agent_id)
	{
		BaseType::operator=(items.begin());
		while(!isFinished() && BaseType::operator->()->agent() != agent_id) BaseType::operator++();
	}

	//! Check if iterator point at the end of plan
	bool isFinished() const
	{
		return ((*this) == items.end());
	}

	//! Increment iterator until it points to next agent's item (postfix)
	PlanItemConstIterator operator++(int) {
		PlanItemConstIterator tmp = (*this);

		do {
			 BaseType::operator++();
		} while(!isFinished() && (BaseType::operator->()->agent() != agent_id));

		return (tmp);
	}

	//! Increment iterator to point to next agent's item (prefix)
	PlanItemConstIterator & operator++() {
		return (*this)++;
	}

	//! Decrement iterator until it points to next agent's item (postfix)
	PlanItemConstIterator operator--(int) {
		PlanItemConstIterator tmp = (*this);

		do {
			 BaseType::operator--();
		} while(!isFinished() && (BaseType::operator->()->agent() != agent_id));

		return (tmp);
	}

	//! Decrement iterator to point to next agent's item (prefix)
	PlanItemConstIterator & operator--() {
		return (*this)--;
	}

	//! Reuse value extraction operators from base type
	using BaseType::operator->;
	using BaseType::operator*;
};

typedef PlanItemConstIterator<Plan::PkmType::ItemSequence> PkmConstIterator;
typedef PlanItemConstIterator<Plan::MbaseType::ItemSequence> MbaseConstIterator;

#endif /* PLANITERATORS_H_ */
