/*
 * position_constraint.cc
 *
 *  Created on: Apr 26, 2010
 *      Author: mboryn
 */

#include "position_constraint.h"

namespace mrrocpp {

namespace ecp {

namespace common {

namespace generator {

position_constraint::position_constraint()
{
}

position_constraint::~position_constraint()
{
}

void position_constraint::set_new_position(const lib::Homog_matrix& new_position)
{
	this->new_position = new_position;
}
const lib::Homog_matrix& position_constraint::get_constrained_position()
{
	return new_position;
}

} // namespace generator

}

}

}
