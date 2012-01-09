/*
 * BlockPlanner.cc
 *
 *  Created on: 2012-01-05
 *      Author: spiatek
 */

#include "BlockPlanner.h"

int BlockPlanner::f(int x, int y, int z) const {
	return 3 - y + width*(3 - x) + width*width*z;
}

int BlockPlanner::x(int f) const {
	return 3 - ((f % (width*width)) / width);
}

int BlockPlanner::y(int f) const {
	return 3 - (f % width);
}

int BlockPlanner::z(int f) const {
	return f / (width*width);
}

int BlockPlanner::f_m(int x_m, int y_m) const {
	return 3 - width*(3 - x_m) + y_m;
}

int BlockPlanner::x_m(int f_m) const {
	return 3 - (f_m / width);
}

int BlockPlanner::y_m(int f_m) const {
	return 3 - (f_m % width);
}

int BlockPlanner::count_f(BlockPosition bp) {
	vector<int> bp_v(3);
	bp_v = bp.getPosition();
	return f(bp_v[0] - 1, bp_v[1] - 1, bp_v[2] - 1);
}

int BlockPlanner::smallest(block_position_list bl) {
	for(int h = 0; h < width*width; h++) {
		int k = x_m(h);
		int l = y_m(h);
		for(block_position_list::iterator it = bl.begin(); it != bl.end(); ++it) {
			int f_val = count_f(*it);
			if(x(f_val) == k and y(f_val) == l and z(f_val) == 0) {
				return f(k, l, 0);
			}
		}
	}
	return -1;
}

BlockPlanner::BlockPlanner(int w, int bc, block_position_list bl) :
		width(w), blocks_count(bc), blocks_list(bl), succ(*this, bc, 0, w*w*MAX_HEIGHT) {

	std::cout << "Block Planner start" << std::endl;

	IntArgs bl_va(bc);

	int counter = 0;
	for(block_position_list::iterator it = bl.begin(); it != bl.end(); ++it) {
		int f_val = count_f(*it);
		bl_va[counter] = IntVar(*this, f_val, f_val).val();
		counter++;
	}

	std::cout << "Block Planner distinct constraint" << std::endl;

	//blocks are pairwise distinct
	distinct(*this, succ);

	std::cout << "Block Planner sort" << std::endl;

	//sort blocks in increasing order
	for(int i = 0; i < blocks_count - 1; i++) {
		rel(*this, succ[i], IRT_LE, succ[i + 1]);
	}

	std::cout << "Block Planner member" << std::endl;

	//get only members of blocks_list

	IntSet is(bl_va);
	for(int i = 0; i < blocks_count; i++) {
		dom(*this, succ[i], is);
	}

	std::cout << "Block Planner first move" << std::endl;

	//first move
	rel(*this, succ[0], IRT_EQ, smallest(bl));

	std::cout << "Block Planner branching" << std::endl;

	branch(*this, succ, INT_VAR_SIZE_MIN, INT_VAL_MIN);
}

BlockPlanner::BlockPlanner(bool share, BlockPlanner& s) : Space(share, s), width(s.width), blocks_count(s.blocks_count), blocks_list(s.blocks_list) {
	succ.update(*this, share, s.succ);
}

void BlockPlanner::print(void) const {
	std::cout << "Plan:" << std::endl;
	for(int i = 0; i < blocks_count; i++) {
		std::cout << "(" << x(succ[i].val()) + 1 << ", " << y(succ[i].val()) + 1
				  << ", "  << z(succ[i].val()) + 1 << ")" << std::endl;
	}
//	std::cout << succ << std::endl;
}

block_position_list BlockPlanner::getPlan(void) {
	block_position_list l;
	for(int i = 0; i < blocks_count; i++) {
		vector<int> pos_v(3);
		pos_v[0] = x(succ[i].val()) + 1;
		pos_v[1] = y(succ[i].val()) + 1;
		pos_v[2] = z(succ[i].val()) + 1;
		int color = -1;
		for(block_position_list::iterator it = blocks_list.begin(); it != blocks_list.end(); ++it) {
			if(succ[i].val() == count_f(*it)) {
				color = (*it).getColor();
				break;
			}
		}
		BlockPosition *bp = new BlockPosition(color, pos_v);
		l.push_back(*bp);
	}
	return l;
}
