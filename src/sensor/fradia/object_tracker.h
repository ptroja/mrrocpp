/*!
 * @file
 * @brief Structure used in the object tracking tasks.
 * 
 * @author  tkornuta
 * @date Aug 4, 2010
 *
 * @ingroup FRADIA_SENSOR
 */


#ifndef OBJECT_TRACKER_H_
#define OBJECT_TRACKER_H_

namespace mrrocpp {
namespace ecp_mp {
namespace sensor {

/**
 * @brief Structure used in the object tracking tasks.
 * @author mboryn
 * @ingroup FRADIA_SENSOR
 */
typedef struct _object_tracker
{
	bool reached;
	bool tracking;
	int x;
	int y;
	int z;
} object_tracker_t;


}// namespace sensor
}// namespace ecp_mp
}// namespace mrrocpp

#endif /* OBJECT_TRACKER_H_ */
