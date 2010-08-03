/*!
 * @file sensor.h
 * @brief File containing sensor interface - a base class for MP, ECP, VSP (and future EDP ) sensors.
 * @author ptrojane <piotr.trojanek@gmail.com>, Warsaw University of Technology
 * @author tkornuta <tkornuta@ia.pw.edu.pl>, Warsaw University of Technology
 */

#if !defined(_SENSOR_H)
#define _SENSOR_H

#include <string>
#include <stdint.h>

#include "lib/com_buf.h"
#include "sensor_error.h"

namespace mrrocpp {
namespace lib {

/**
 * @brief Commands send to VSP.
 * @author tkornuta
 */
typedef enum _VSP_COMMAND
{
	VSP_CONFIGURE_SENSOR, VSP_INITIATE_READING, VSP_GET_READING, VSP_TERMINATE
} VSP_COMMAND_t;

/**
 * @brief VSP responses.
 * @author tkornuta
 */
typedef enum _VSP_REPORT
{
	VSP_REPLY_OK, VSP_SENSOR_NOT_CONFIGURED, VSP_READING_NOT_READY, INVALID_VSP_COMMAND
} VSP_REPORT_t;

/**
 * @brief Structure used in the cube grasping task.
 */
typedef struct _object_tracker
{
	bool reached;
	bool tracking;
	int x;
	int y;
	int z;
} object_tracker_t;

/**
 * @brief Structure used in the cube state recognition task.
 */
typedef struct _cube_face
{
	char colors[9];
} cube_face_t;

/**
 * @brief Structure used during the visual servoing.
 */
typedef struct _vis_sac
{
	double frame_O_T_G[16];
	double frame_E_T_G[16];
	double frame_E_r_G[6];
	double frame_E_r_G__CEIH[6];
	double frame_E_r_G__f[6];
	double fEIH_G[8];
} vis_sac_t;

/**
 * @brief Empty data structure.
 */
typedef struct _empty
{
	//! This is empty data type
} empty_t;

/**
 * Sensor names type.
 */
typedef std::string SENSOR_t;

const SENSOR_t SENSOR_CAMERA_SA = "SENSOR_CAMERA_SA";
const SENSOR_t SENSOR_CAMERA_ON_TRACK = "SENSOR_CAMERA_ON_TRACK";
const SENSOR_t SENSOR_CAMERA_POSTUMENT = "SENSOR_CAMERA_POSTUMENT";


/**
 * @brief Base class for MP, ECP, VSP (and future EDP ) sensors.
 * @author tkornuta
 * @author ptrojane
 */
class sensor_interface
{
public:
	/**
	 * Abstract method responsible for reading retrieval.
	 */
	virtual void get_reading(void) = 0;

	/**
	 * Virtual method responsible for sensor configuration.
	 */
	virtual void configure_sensor(void)
	{
	}

	/**
	 * Virtual method responsible for reading initialization.
	 */
	virtual void initiate_reading(void)
	{
	}

	/**
	 * Virtual destructor. Empty.
	 */
	virtual ~sensor_interface()
	{
	}
};

} // namespace lib
} // namespace mrrocpp

#endif /* _SENSOR_H */
