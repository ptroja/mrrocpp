/*
 * JntArray.h
 *
 *  Created on: Jan 6, 2010
 *      Author: konradb3
 */

#ifndef JNTARRAY_H_
#define JNTARRAY_H_

#include <vector>

namespace mrrocpp {
namespace lib {
class JntArray : public std::vector<double>
{
public:
  JntArray() {}
  JntArray(int size) : std::vector<double> (size) {}
  JntArray(double *ptr, size_t n) : std::vector<double> (ptr, ptr+n) {}
};

}
}
#endif /* JNTARRAY_H_ */
