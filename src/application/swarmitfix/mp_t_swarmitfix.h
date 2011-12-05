#if !defined(__MP_T_SWARMITFIX_H)
#define __MP_T_SWARMITFIX_H

#include <boost/shared_ptr.hpp>
#include <boost/unordered_map.hpp>

#include "base/mp/mp_task.h"
#include "base/lib/impconst.h"

#include "base/lib/swarmtypes.h"
#include "robot/spkm/dp_spkm.h"
#include "planner.h"


namespace mrrocpp {
namespace mp {
namespace task {

/** @defgroup swarmitfix swarmitfix
 *  @ingroup application
 *  @{
 */

typedef struct _IO {

	//! Composite data type for inter-agent communication subsystem 
	typedef struct _transmitters {
	
		//! Composite data type for transmitter's data buffers  
		typedef struct _spkm2 {
		
			//! Input buffers
			struct _inputs { 
		
				//! Data type of input buffer 
				typedef InputPtr<lib::notification_t> notification_t;
	
				//! InputBuffer: Status of the agent
				notification_t notification;
				
			} inputs;
			
			//! Output buffers
			struct _outputs {
			//! Data type of output buffer
			typedef OutputPtr<lib::spkm::next_state_t> command_t;

			//! OutputBuffer: Command to execute
			command_t command;
			
			} outputs;
			
		} spkm2_t;
		
		//! Transmitter's data structure itself
		spkm2_t spkm2;
		
		//! Composite data type for transmitter's data buffers  
		typedef struct _smb2 {
		
			//! Input buffers
			struct _inputs { 
		
				//! Data type of input buffer 
				typedef InputPtr<lib::notification_t> notification_t;
	
				//! InputBuffer: Status of the agent
				notification_t notification;
				
			} inputs;
			
			//! Output buffers
			struct _outputs {
			} outputs;
			
		} smb2_t;
		
		//! Transmitter's data structure itself
		smb2_t smb2;
		
		//! Composite data type for transmitter's data buffers  
		typedef struct _spkm1 {
		
			//! Input buffers
			struct _inputs { 
		
				//! Data type of input buffer 
				typedef InputPtr<lib::notification_t> notification_t;
	
				//! InputBuffer: Status of the agent
				notification_t notification;
				
			} inputs;
			
			//! Output buffers
			struct _outputs {
			//! Data type of output buffer
			typedef OutputPtr<lib::spkm::next_state_t> command_t;

			//! OutputBuffer: Command to execute
			command_t command;
			
			} outputs;
			
		} spkm1_t;
		
		//! Transmitter's data structure itself
		spkm1_t spkm1;
		
		//! Composite data type for transmitter's data buffers  
		typedef struct _smb1 {
		
			//! Input buffers
			struct _inputs { 
		
				//! Data type of input buffer 
				typedef InputPtr<lib::notification_t> notification_t;
	
				//! InputBuffer: Status of the agent
				notification_t notification;
				
			} inputs;
			
			//! Output buffers
			struct _outputs {
			} outputs;
			
		} smb1_t;
		
		//! Transmitter's data structure itself
		smb1_t smb1;
		

	} transmitters_t;
	
	//! Inter-agent communication subsystem itself
	transmitters_t transmitters;
	

	//! Composite data type for sensory subsystem 
	typedef struct _sensors {
	
		//! Composite data type for transmitter's data buffers  
		typedef struct _planner {
		
			//! Input buffers
			struct _inputs { 
		
				//! Data type of input buffer 
				typedef InputPtr<lib::empty_t> trigger_t;
	
				//! InputBuffer: Trigger execution of the next plan step
				trigger_t trigger;
				
			} inputs;
			
			//! Output buffers
			struct _outputs {
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
typedef enum _PLAN_STATUS { ONGOING, FAILURE } PlanStatus;

//! Type for worker (ECP) agent status
typedef enum _WORKER_STATUS { IDLE, BUSY } WorkerStatus;

//! Associative container type for worker status
typedef boost::unordered_map<const lib::robot_name_t, WorkerStatus> WorkersStatus;

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
	

public:
	//! Constructor
	swarmitfix(lib::configurator &_config);

	/// utworzenie robotow
	void create_robots(void);

	// methods for mp template
	void main_task_algorithm(void);

	//! Alternative task for testing (reuses the same I/O buffers as the final task)
	void main_test_algorithm(void);
};

/** @} */// end of swarmitfix

} // namespace task
} // namespace mp
} // namespace mrrocpp

#endif /* __MP_T_SWARMITFIX_H */
