#if !defined(_SMB_CONST_H)
#define _SMB_CONST_H

/*!
 * @file
 * @brief File contains constants and structures for SwarmItFix Mobile Base
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup smb
 */

#include "base/lib/impconst.h"

namespace mrrocpp {
namespace lib {
namespace smb {

/*!
 * @brief SwarmItFix Mobile Base total number of servos
 * @ingroup smb
 */
const int NUM_OF_SERVOS = 2;

/*!
 * @brief SwarmItFix Mobile Base total number of legs
 * @ingroup smb
 */
const int LEG_CLAMP_NUMBER = 3;


/*!
 * @brief SwarmItFix Mobile Base leg position variants from all legs point of view
 * @ingroup smb
 */
typedef enum _ALL_LEGS_VARIANT
{
	ALL_DOWN, ALL_UP, ONE_UP_TWO_DOWN, TWO_UP_ONE_DOWN
} ALL_LEGS_VARIANT;

/*!
 * @brief SwarmItFix Mobile Base EDP command buffer variant enum
 * @ingroup smb
 */
enum CBUFFER_VARIANT
{
	POSE, QUICKSTOP, CLEAR_FAULT, FESTO
};

/*!
 * Pose specification variants
 * @ingroup smb
 */
typedef enum _POSE_SPECIFICATION
{
	FRAME, JOINT, MOTOR
} POSE_SPECIFICATION;


} // namespace smb
} // namespace lib
} // namespace mrrocpp

#endif /* _SMB_CONST_H */
