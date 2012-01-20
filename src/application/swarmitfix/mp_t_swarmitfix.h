#if !defined(__MP_T_SWARMITFIX_H)
#define __MP_T_SWARMITFIX_H

#include <boost/shared_ptr.hpp>
#include <string>

#include "WorkersStatus.h"

#include "base/mp/mp_task.h"
#include "base/lib/impconst.h"

#include "base/lib/swarmtypes.h"
#include "robot/shead/dp_shead.h"
#include "robot/spkm/dp_spkm.h"
#include "robot/smb/dp_smb.h"
#include "robot/sbench/dp_sbench.h"
#include "planner.h"

#include "base/ecp_mp/ecp_ui_msg.h"
#include "base/mp/mp_exceptions.h"

#include "plan.hxx"

namespace mrrocpp {
namespace mp {
namespace task {

/** @defgroup swarmitfix swarmitfix
 *  @ingroup application
 *  @{
 */

typedef struct _IO
{

	//! Composite data type for inter-agent communication subsystem 
	typedef struct _transmitters
	{

		//! Common input buffers
		typedef struct _inputs
		{

			//! Data type of input buffer
			typedef InputPtr <lib::notification_t> notification_t;

			//! InputBuffer: Status of the agent
			notification_t notification;

		} inputs_t;

		//! Composite data type for transmitter's data buffers  
		typedef struct _spkm
		{

			//! Input buffers
			inputs_t inputs;

			//! Output buffers
			struct _outputs
			{
				//! Data type of output buffer
				typedef OutputPtr <lib::spkm::next_state_t> command_t;

				//! OutputBuffer: Command to execute
				command_t command;

			} outputs;

		} spkm_t;

		//! Transmitter's data structure itself

		//! Composite data type for transmitter's data buffers  
		typedef struct _smb
		{

			//! Input buffers
			inputs_t inputs;

			//! Output buffers
			struct _outputs
			{
				//! Data type of output buffer
				typedef OutputPtr <lib::smb::next_state_t> command_t;

				//! OutputBuffer: Command to execute
				command_t command;

			} outputs;

		} smb_t;

		//! Composite data type for transmitter's data buffers
		typedef struct _shead
		{

			//! Input buffers
			inputs_t inputs;

			//! Output buffers
			struct _outputs
			{
				//! Data type of output buffer
				typedef OutputPtr <lib::shead::next_state> command_t;

				//! OutputBuffer: Command to execute
				command_t command;

			} outputs;

		} shead_t;

		//! Composite data type for transmitter's data buffers
		typedef struct _sbench
		{

			//! Input buffers
			inputs_t inputs;

			//! Output buffers
			struct _outputs
			{
				//! Data type of output buffer
				typedef OutputPtr <lib::sbench::cbuffer> command_t;

				//! OutputBuffer: Command to execute
				command_t command;

			} outputs;

		} sbench_t;

		//! Transmitter's data structures itself
		smb_t smb1, smb2;
		spkm_t spkm1, spkm2;
		shead_t shead1, shead2;
		sbench_t sbench;

	} transmitters_t;

	//! Inter-agent communication subsystem itself
	transmitters_t transmitters;

	//! Composite data type for sensory subsystem 
	typedef struct _sensors
	{

		//! Composite data type for transmitter's data buffers  
		typedef struct _planner
		{

			//! Input buffers
			struct _inputs
			{

				//! Data type of input buffer 
				typedef InputPtr <lib::empty_t> trigger_t;

				//! InputBuffer: Trigger execution of the next plan step
				trigger_t trigger;

			} inputs;

			//! Output buffers
			struct _outputs
			{
			} outputs;

		} planner_t;

		//! Transmitter's data structure itself
		planner_t planner;

	} sensors_t;

	//! Sensory subsystem itself
	sensors_t sensors;
} IO_t;

class swarmitfix : public task
{
private:
	// Start of user code Internal memory variables
//! Type for plan realization status
	typedef enum _PLAN_STATUS
	{
		ONGOING, FAILURE
	} PlanStatus;

//! Planner object
	planner pp;

// End of user code

//! Internal memory variable: Current plan status
	PlanStatus current_plan_status;
	//! Internal memory variable: Current workers status	
	WorkersStatus current_workers_status;

	//! Input/Output subsystems
	IO_t IO;

	//! Initial condition: Execute if the status in non-failure	  
	bool b1_initial_condition(void) const;

	//! Terminal condition: Terminate in case of failure	  
	bool b1_terminal_condition(void) const;

	//! Transition function: Progress with the next plan command
	void b1_plan_progress(void);

	//! Transition function: Record notification message
	void b1_handle_spkm1_notification(void);

	//! Transition function: Record notification message
	void b1_handle_spkm2_notification(void);

	//! Transition function: Record notification message
	void b1_handle_smb1_notification(void);

	//! Transition function: Record notification message
	void b1_handle_smb2_notification(void);

	//! Initial condition: Execute in case of emergency	  
	bool b2_initial_condition(void) const;

	//! Terminal condition: Do not terminate (requires operator to restart the task)	  
	bool b2_terminal_condition(void) const;

	//! Transition function: Send stop commands to all the busy robots
	void b2_stop_all(void);

	//! Alternative task for testing (reuses the same I/O buffers as the final task)
	void main_test_algorithm(void);

	//! Helper function
	void handleNotification(const lib::robot_name_t & robot_name, IO_t::transmitters_t::inputs_t & inputs);

	//! Execute PKM+HEAD plan item
	void executeCommandItem(const Plan::PkmType::ItemType & pkmCmd, IO_t & IO);

	//! Execute MB+BENCH plan item
	void executeCommandItem(const Plan::MbaseType::ItemType & smbCmd, IO_t IO);

	//! Save modified plan to file
	void save_plan(const Plan & p);

	//! Step-mode execution of pkm item
	lib::UI_TO_ECP_REPLY step_mode(Pkm::ItemType & item);

	//! Step-mode execution of mbase item
	lib::UI_TO_ECP_REPLY step_mode(Mbase::ItemType & item);

	//! Access to plan items at given index
	template <typename T>
	typename T::iterator StateAtInd(int ind, T & items)
	{
		typename T::iterator it = items.begin();

		while ((it != items.end()) && it->ind() != ind)
			++it;

		return it;
	}

public:
	//! Constructor
	swarmitfix(lib::configurator &_config);

	/// utworzenie robotow
	void create_robots(void);

	// methods for mp template
	void main_task_algorithm(void);
};

/** @} */ // end of swarmitfix
}// namespace task
} // namespace mp
} // namespace mrrocpp

#endif /* __MP_T_SWARMITFIX_H */
