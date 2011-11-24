/*
 * BlockPosition.h
 *
 *  Created on: 22-11-2011
 *      Author: spiatek
 */

#ifndef BLOCKPOSITION_H_
#define BLOCKPOSITION_H_

#include <vector>
#include <string>

using namespace std;

class BlockPosition
{

public:

	BlockPosition()
	{
	}

	BlockPosition(int c, vector <int> p) {
		color = c;
		position = p;
	}

	virtual ~BlockPosition()
	{
	}

	int getColor() {
		return color;
	}

	vector <int> getPosition() {
		return position;
	}

private:

	int color;
	std::vector <int> position;

};

#endif /* BLOCKPOSITION_H_ */
