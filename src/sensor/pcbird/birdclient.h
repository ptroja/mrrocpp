/**
 * @file
 * @brief File containing declarations of structure for storing data retrieved from the PcBird sensor and low-level (hardware access) functions.
 *
 * @author B.Bielawski
 * @author T.Wlostowski
 * @author T.Adamczyk
 * @author tkornuta
 * @date 16.06.2008
 *
 * @ingroup PCBIRD_SENSOR
 */

#ifndef __BIRDCLIENT_H
#define __BIRDCLIENT_H

#include <stdint.h>

namespace mrrocpp {
namespace ecp_mp {
namespace sensor {

/**
 * @brief Structure for storing position and orientation of the PcBird other transceiver in relation to its base.
 *
 * The pose is stored in the form of Euler Angles (azimuth, elevation, roll).
 *
 * @author B.Bielawski
 * @author T.Wlostowski
 * @author T.Adamczyk
 *
 * @ingroup PCBIRD_SENSOR
 */
typedef struct pcbird_pos_t
{
	/** @brief X coordinate.*/
	float x;

	/** @brief Y coordinate.*/
	float y;

	/** @brief Z coordinate.*/
	float z;

	/** @brief Azimuth angle.*/
	float a;

	/** @brief Elevation angle.*/
	float b;

	/** @brief Roll angle.*/
	float g;

	/** @brief Distance.*/
	float distance;

	/** @brief Timestamp - seconds part. */
	uint32_t ts_sec;

	/** @brief Timestamp - microseconds part. */
	uint32_t ts_usec;
} pcbird_pos;

/** @ingroup PCBIRD
 * \{
 */

/**
 * @brief Connects with the PcBird process.
 * @param addr Node name.
 * @param port Socket port.
 * @return Socket descriptor.
 *
 * @author B.Bielawski
 * @author T.Wlostowski
 * @author T.Adamczyk
 */
int pcbird_connect(const char *addr, unsigned short port);

/**
 * @brief Ends the PcBird connection.
 * @param fd Socket descriptor.
 *
 * @author B.Bielawski
 * @author T.Wlostowski
 * @author T.Adamczyk
 */
void pcbird_disconnect(int fd);

/**
 * @brief Starts data streaming.
 * @param fd Socket descriptor.
 * @return Operation status.
 */
int pcbird_start_streaming(int fd);

/**
 * @brief Ends data streaming.
 * @param fd
 * @return Operation status.
 *
 * @author B.Bielawski
 * @author T.Wlostowski
 * @author T.Adamczyk
 */
int pcbird_stop_streaming(int fd);

/**
 * @brief Retrieval of single pose from the PcBird.
 * @param fd Socket descriptor.
 * @param p
 * @return Operation status.
 *
 * @author B.Bielawski
 * @author T.Wlostowski
 * @author T.Adamczyk
 */
int pcbird_get_single_position(int fd, pcbird_pos_t *p);

/**
 * @brief Checks whether new data arrived.
 *
 * Used in the streaming mode.
 *
 * @param fd Socket descriptor.
 * @return Operation status.
 *
 * @author B.Bielawski
 * @author T.Wlostowski
 * @author T.Adamczyk
 */
int pcbird_data_avail(int fd);

/**
 * @brief Unblocking position retrieval.
 *
 * Used in the streaming mode.
 *
 * @param fd Socket descriptor.
 * @param p
 * @return Operation status.
 *
 * @author B.Bielawski
 * @author T.Wlostowski
 * @author T.Adamczyk
 */
int pcbird_get_streaming_position(int fd, pcbird_pos_t *p);

/** \} */

} // namespace sensor
} // namespace ecp_mp
} // namespace mrrocpp


#endif
