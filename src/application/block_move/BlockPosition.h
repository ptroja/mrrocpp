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
#include <iostream>

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

	int getValue() {
		return 0;
	}

	void print() {
		string w_col;
		switch(color) {
			case 0:
				w_col = "undefined";
				break;
			case 1:
				w_col = "blue";
				break;
			case 2:
				w_col = "red";
				break;
			case 3:
				w_col = "green";
				break;
			case 4:
				w_col = "yellow";
				break;
		}
		cout << "Color: " << w_col << ", Position: " << position[0] << ", " << position[1] << ", " << position[2] << endl;
		return;
	}

	bool operator== (BlockPosition &bp) {
		if (this->getPosition()[0] == bp.getPosition()[0] &&
			this->getPosition()[1] == bp.getPosition()[1] &&
			this->getPosition()[2] == bp.getPosition()[2]) {
			return true;
		}
		return false;
	}

	bool operator< (BlockPosition &bp) {
		if(this->getPosition()[2] > bp.getPosition()[2]) {
			return true;
		}
		else if(this->getPosition()[2] == bp.getPosition()[2]) {
			if(this->getPosition()[1] > bp.getPosition()[1]) {
				return true;
			}
			else if(this->getPosition()[1] == bp.getPosition()[1]) {
				if(this->getPosition()[0] > bp.getPosition()[0]) {
					return true;
				}
			}
		}
		return false;
	}

	bool operator> (BlockPosition &bp) {
		if(this->getPosition()[2] < bp.getPosition()[2]) {
			return true;
		}
		else if(this->getPosition()[2] == bp.getPosition()[2]) {
			if(this->getPosition()[1] < bp.getPosition()[1]) {
				return true;
			}
			else if(this->getPosition()[1] == bp.getPosition()[1]) {
				if(this->getPosition()[0] < bp.getPosition()[0]) {
					return true;
				}
			}
		}
		return false;
	}

private:

	int color;
	std::vector <int> position;

};

#endif /* BLOCKPOSITION_H_ */
