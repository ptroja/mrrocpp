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

	BlockPosition(const char *c, vector <int> p) {
		color = c;
		position = p;
	}

	virtual ~BlockPosition()
	{
	}

	const char* getColor() {
		return color;
	}

	vector <int> getPosition() {
		return position;
	}

private:

	const char* color;
	std::vector <int> position;

};

#endif /* BLOCKPOSITION_H_ */
