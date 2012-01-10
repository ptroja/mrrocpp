#if !defined(_SBENCH_CONST_H)
#define _SBENCH_CONST_H

/*!
 * @file
 * @brief File contains constants and structures for SwarmItFix bench
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup sbench
 */

#include "base/lib/impconst.h"

#define FESTO_C1_GROUP 1
#define FESTO_C1_BIT (1<<0)
#define FESTO_C1_BIT_TO_SET 0

#define FESTO_C3_GROUP 2
#define FESTO_C3_BIT (1<<1)
#define FESTO_C3_BIT_TO_SET 1

#define FESTO_C2_GROUP 2
#define FESTO_C2_BIT (1<<0)
#define FESTO_C2_BIT_TO_SET 0

#define FESTO_CY11_GROUP 1
#define FESTO_CY11_BIT (1<<3)
#define FESTO_CY11_BIT_TO_SET 3

#define FESTO_CY12_GROUP 1
#define FESTO_CY12_BIT (1<<2)
#define FESTO_CY12_BIT_TO_SET 2

#define FESTO_CY31_GROUP 1
#define FESTO_CY31_BIT (1<<5)
#define FESTO_CY31_BIT_TO_SET 5

#define FESTO_CY32_GROUP 1
#define FESTO_CY32_BIT (1<<4)
#define FESTO_CY32_BIT_TO_SET 4

#define FESTO_CY21_GROUP 1
#define FESTO_CY21_BIT (1<<7)
#define FESTO_CY21_BIT_TO_SET 7

#define FESTO_CY22_GROUP 1
#define FESTO_CY22_BIT (1<<6)
#define FESTO_CY22_BIT_TO_SET 6

#define FESTO_CH1_GROUP 2
#define FESTO_CH1_BIT (1<<5)
#define FESTO_CH1_BIT_TO_SET 5

#define FESTO_CH3_GROUP 2
#define FESTO_CH3_BIT (1<<3)
#define FESTO_CH3_BIT_TO_SET 3

#define FESTO_CH2_GROUP 2
#define FESTO_CH2_BIT (1<<4)
#define FESTO_CH2_BIT_TO_SET 4

// activation not important in last version

#define FESTO_A1_GROUP 1
#define FESTO_A1_BIT (1<<1)
#define FESTO_A1_BIT_TO_SET 1

#define FESTO_A3_GROUP 2
#define FESTO_A3_BIT (1<<2)
#define FESTO_A3_BIT_TO_SET 2

#define FESTO_A2_GROUP 2
#define FESTO_A2_BIT (1<<7)
#define FESTO_A2_BIT_TO_SET 7

#define FESTO_H1_GROUP 2
#define FESTO_H1_BIT (1<<6)
#define FESTO_H1_BIT_TO_SET 6

namespace mrrocpp {
namespace lib {
namespace sbench {

/*!
 * @brief SwarmItFix Bench robot label
 * @ingroup sbench
 */
const robot_name_t ROBOT_NAME = "sbench";

/*!
 * @brief SwarmItFix Bench total number of pins
 * @ingroup sbench
 */
const int NUM_OF_PINS = 64;

/*!
 * @brief SwarmItFix Bench total number of servos
 * @ingroup sbench
 */
const int NUM_OF_SERVOS = 0;

} // namespace sbench
} // namespace lib
} // namespace mrrocpp

#endif /* _SBENCH_CONST_H */
