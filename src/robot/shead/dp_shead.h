#if !defined(__SHEAD_DATA_PORT_H)
#define __SHEAD_DATA_PORT_H

/*!
 * @file
 * @brief File contains data port communication structures for SwarmItFix Head
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup shead
 */

#include <string>

namespace mrrocpp {
namespace lib {

enum SHEAD_STATE_OF_THE_HEAD
{
	HEAD_STATE_SOLDIFIED, HEAD_STATE_DESOLDIFIED, HEAD_STATE_INTERMEDIATE
};

enum SHEAD_STATE_OF_THE_VACUUM
{
	VACUUM_STATE_ON, VACUUM_STATE_OFF, VACUUM_STATE_INTERMEDIATE
};

enum SHEAD_HEAD_SOLIDIFICATION
{
	SOLIDIFY, DESOLIDIFY, SHEAD_HEAD_SOLIDIFICATION_NO_ACTION
}; // namespace mrrocpp

enum SHEAD_VACUUM_ACTIVATION
{
	VACUUM_ON, VACUUM_OFF, SHEAD_VACUUM_ACTIVATION_NO_ACTION
}; // namespace mrrocpp


const std::string SHEAD_HEAD_SOLIDIFICATION_DATA_PORT = "SHEAD_HEAD_SOLIDIFICATION_DATA_PORT";
const std::string SHEAD_VACUUM_ACTIVATION_DATA_PORT = "SHEAD_VACUUM_ACTIVATION_DATA_PORT";
const std::string SHEAD_REPLY_DATA_REQUEST_PORT = "SHEAD_REPLY_DATA_REQUEST_PORT";

struct shead_reply
{
	SHEAD_STATE_OF_THE_HEAD head_state;
	SHEAD_STATE_OF_THE_VACUUM vacuum_state;

};

}
}

#endif
