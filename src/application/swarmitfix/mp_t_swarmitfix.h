#if !defined(__MP_T_SWARMITFIX_H)
#define __MP_T_SWARMITFIX_H

#include <boost/shared_ptr.hpp>
#include <boost/unordered_map.hpp>
#include <boost/ptr_container/ptr_unordered_map.hpp>

#include "base/lib/impconst.h"
#include "base/lib/swarmtypes.h"

namespace mrrocpp {
namespace mp {
namespace task {

/** @defgroup swarmitfix swarmitfix
 *  @ingroup application
 *  @{
 */

typedef struct _IO {
	typedef struct _transmitters {
		typedef struct _spkm1 {
			typedef InputPtr<lib::notification_t> notificationBuffer_t;

			notificationBuffer_t notification;

			typedef OutputPtr<lib::notification_t> command_t;

			command_t command;
		} spkm1_t;
		spkm1_t spkm1;

		typedef struct _spkm2 {
			typedef InputPtr<lib::notification_t> notificationBuffer_t;

			notificationBuffer_t notification;

			typedef OutputPtr<lib::notification_t> command_t;

			command_t command;
		} spkm2_t;
		spkm1_t spkm2;

		typedef struct _smb1 {
			typedef InputPtr<lib::notification_t> notificationBuffer_t;

			notificationBuffer_t notification;

			typedef OutputPtr<lib::notification_t> command_t;

			command_t command;
		} smb1_t;
		smb1_t smb1;

		typedef struct _smb2 {
			typedef InputPtr<lib::notification_t> notificationBuffer_t;

			notificationBuffer_t notification;

			typedef OutputPtr<lib::notification_t> command_t;

			command_t command;
		} smb2_t;
		smb2_t smb2;

	} transmitters_t;
	transmitters_t transmitters;
} IO_t;

class swarmitfix : public task
{
private:
	//! Type for plan realization status
	typedef enum _PLAN_STATUS { ONGOING, FAILURE } PlanStatus;

	//! Type for worker (ECP) agent status
	typedef enum _WORKER_STATUS { IDLE, BUSY } WorkerStatus;

	//! Associative container type for worker status
	typedef boost::unordered_map<const lib::robot_name_t, WorkerStatus> WorkersStatus;

	//! Current plan status
	PlanStatus current_plan_status;

	//! Current workers status
	WorkersStatus current_workers_status;

	//! Input/Output subsystems
	IO_t IO;

public:
	//! Constructor
	swarmitfix(lib::configurator &_config);

	/// utworzenie robotow
	void create_robots(void);

	// methods for mp template
	void main_task_algorithm(void);
};

/** @} */// end of swarmitfix

} // namespace task
} // namespace mp
} // namespace mrrocpp

#endif
