/*!
 * \file kinematic_parameters_spkm.h
 * \brief File containing the declaration of kinematic_parameters class.
 *
 * \author tkornuta
 * \date Jan 5, 2010
 */

#ifndef KINEMATIC_PARAMETERS_H_
#define KINEMATIC_PARAMETERS_H_

namespace mrrocpp {
namespace kinematics {
namespace spkm {

/*!
 * \class kinematic_parameters_spkm
 * \brief Class storing parameters for PKM and spherical wrist attached to it.
 *
 * \author tkornuta
 * \date Jan 05, 2010
 */
class kinematic_parameters_spkm
{
	public:
		//! Constructor.
		kinematic_parameters_spkm();

		//! Virtual destructor - empty.
		virtual ~kinematic_parameters_spkm() { }
};

} // namespace spkm
} // namespace kinematic
} // namespace mrrocpp

#endif /* KINEMATIC_PARAMETERS_H_ */
