#ifndef __ECP_WIIMOTE_H
#define __ECP_WIIMOTE_H

#include <netdb.h>
#include <netinet/in.h>
#include "base/ecp_mp/ecp_mp_sensor.h"

namespace mrrocpp {
namespace ecp_mp {
namespace sensor {

/** @addtogroup wii_teach
 *
 *  @{
 */

//Structure for storing data retrieved from the Wii-mote server
typedef struct _wiimote_t
{
	int left;
	int right;
	int up;
	int down;
    int buttonA;
    int buttonB;
    int button1;
    int button2;
    int buttonPlus;
    int buttonMinus;
    int buttonHome;
	float orientation_x;
	float orientation_y;
	float orientation_z;
} wiimote_t;

typedef struct _wii_command
{
  bool led_change;
  unsigned int led_status;
  bool rumble;
} wii_command_t;

const lib::sensor::SENSOR_t SENSOR_WIIMOTE = "SENSOR_WIIMOTE";

/**
 * Virtual sensor that communicates with the Wii-mote
 *
 * @author jedrzej
 */
class wiimote : public sensor_interface
{
private:
	//socket descriptor
	int sockfd;

	//address of the wii-mote server
	sockaddr_in serv_addr;

	//pointer to the server
	hostent* server;

	//link to SRP communication object
	lib::sr_ecp& sr_ecp_msg;

	//sensor name
	const lib::sensor::SENSOR_t sensor_name;

	struct _to_vsp {
		lib::sensor::VSP_COMMAND_t i_code;
		wii_command_t wii_command;
	} to_vsp;

	struct _from_vsp {
		lib::sensor::VSP_REPORT_t vsp_report;
		wiimote_t wiimote;
	} from_vsp;

public:
	wiimote_t image;

	/**
	 * Creates the sensor object. Connects to the Wii-mote server
	 *
	 * @param _sensor_name 		the name f the sensor
	 * @param _section_name		the name of the section in the config file
	 * @param _ecp_mp_object
	 * @param _union_size
	 *
	 * @author jedrzej
	 */
	wiimote (lib::sensor::SENSOR_t _sensor_name, const std::string & _section_name, lib::sr_ecp & _sr_ecp_msg, lib::configurator & config);

	/**
	 * Sends sensor configuration to the Wii-mote server
	 *
	 * @author jedrzej
	 */
	void configure_sensor (void);

	/**
	 * Sends "initiate reading" command to the Wii-mote server
	 *
	 * @author jedrzej
	 */
	void initiate_reading (void);

	/**
	 * Sends given command to the Wii-mote server
	 *
	 * @author jedrzej
	 */
	void send_reading (const wii_command_t & cmd);

	/**
	 * Retrieves aggregated data from the Wii-mote server
	 *
	 * @author jedrzej
	 */
	void get_reading (void);

	void get_reading (const wii_command_t & msg);

	/**
	 * Terminates the connection to the Wii-mote server
	 *
	 * @author jedrzej
	 */
	~wiimote();
};

/** @} */ // end of wii_teach

} // namespace sensor
} // namespace ecp_mp
} // namespace mrrocpp

#endif
