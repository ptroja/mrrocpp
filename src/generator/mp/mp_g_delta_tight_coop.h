#ifndef MP_GEN_COMMON_H_
#define MP_GEN_COMMON_H_

/*!
 * @file
 * @brief File contains mp delta and tight coop generators declaration
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup generators
 */

#include "base/mp/generator/mp_generator.h"

namespace mrrocpp {
namespace mp {
namespace generator {

class delta : public generator
{
public:
	delta(task::task& _mp_task);
	lib::trajectory_description irp6ot_td;
	lib::trajectory_description irp6p_td;
};

// ####################################################################################################
// Generator prostoliniowy o zadany przyrost polozenia/orientacji
// ####################################################################################################

class tight_coop : public delta
{
public:
			tight_coop(task::task& _mp_task, lib::trajectory_description irp6ot_tr_des, lib::trajectory_description irp6p_tr_des);

	virtual bool first_step();

	virtual bool next_step();
};

} // namespace generator
} // namespace mp
} // namespace mrrocpp

#endif /*MP_GENERATORS_H_*/
