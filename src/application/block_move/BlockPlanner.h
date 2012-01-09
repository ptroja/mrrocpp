/*
 * BlockPlanner.h
 *
 *  Created on: 2012-01-06
 *      Author: spiatek
 */

#ifndef BLOCKPLANNER_H_
#define BLOCKPLANNER_H_

#define WIDTH 4
#define MAX_HEIGHT 5

#include <vector>
#include <list>

#include <gecode/int.hh>
#include <gecode/gist.hh>

#include "BlockPosition.h"

typedef list<BlockPosition> block_position_list;

using namespace Gecode;

class BlockPlanner : public Space {

protected:

	const int width;
	const int blocks_count;

	block_position_list blocks_list;
	IntVarArray succ;

public:

	int f(int, int, int) const;
	int x(int) const;
	int y(int) const;
	int z(int) const;
	int f_m(int, int) const;
	int x_m(int) const;
	int y_m(int) const;

	int count_f(BlockPosition);
	int smallest(block_position_list);

	BlockPlanner(int, int, block_position_list);
	BlockPlanner(bool, BlockPlanner&);

	virtual Space* copy(bool share) {
		return new BlockPlanner(share, *this);
	}

	void print(void) const;
	block_position_list getPlan(void);
};

#endif /* BLOCKPLANNER_H_ */
