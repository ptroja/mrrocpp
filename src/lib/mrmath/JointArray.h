/*
 * JntArray.h
 *
 *  Created on: Jan 6, 2010
 *      Author: konradb3
 */

#ifndef JOINTARRAY_H_
#define JOINTARRAY_H_

#include <vector>

namespace mrrocpp {
namespace lib {

class JointArray : public std::vector<double>
{
public:
	JointArray() {}
	JointArray(int size) : std::vector<double> (size) {}
	JointArray(const double *ptr, size_t n) : std::vector<double> (ptr, ptr+n) {}
};

}
}
#endif /* JNTARRAY_H_ */
