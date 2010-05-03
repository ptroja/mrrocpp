/*
 * generator/ecp_g_smb.h
 *
 *Author: yoyek
 */

#ifndef ECP_G_SMB_H_
#define ECP_G_SMB_H_

#include <time.h>
#include "ecp/common/generator/ecp_generator.h"
#include "lib/data_port_headers/smb.h"

namespace mrrocpp {
namespace ecp {
namespace smb {
namespace generator {

class pin_lock: public common::generator::generator {
private:
	lib::smb_multi_pin_locking mp_ecp_smb_multi_pin_locking_structure;

	lib::single_thread_port<lib::smb_multi_pin_locking>
			* smb_multi_pin_locking_data_port;
	lib::smb_multi_pin_locking smb_multi_pin_locking_structure;

	lib::single_thread_request_port<lib::smb_multi_leg_reply>
			* smb_multi_leg_reply_data_request_port;
	lib::smb_multi_leg_reply edp_ecp_smb_multi_leg_reply_structure;

public:
	pin_lock(common::task::task& _ecp_task); //constructor
	bool first_step(); //first step generation
	bool next_step(); //next step generation

	void create_ecp_mp_reply();
	void get_mp_ecp_command();

};

class pin_unlock: public common::generator::generator {
private:
	lib::smb_multi_pin_locking mp_ecp_smb_multi_pin_locking_structure;

	lib::single_thread_port<lib::smb_multi_pin_locking>
			* smb_multi_pin_locking_data_port;
	lib::smb_multi_pin_locking smb_multi_pin_locking_structure;

	lib::single_thread_request_port<lib::smb_multi_leg_reply>
			* smb_multi_leg_reply_data_request_port;
	lib::smb_multi_leg_reply edp_ecp_smb_multi_leg_reply_structure;

public:
	pin_unlock(common::task::task& _ecp_task); //constructor
	bool first_step(); //first step generation
	bool next_step(); //next step generation

	void create_ecp_mp_reply();
	void get_mp_ecp_command();

};

class pin_rise: public common::generator::generator {
private:
	lib::smb_multi_pin_insertion mp_ecp_smb_multi_pin_insertion_structure;

	lib::single_thread_port<lib::smb_multi_pin_insertion>
			* smb_multi_pin_insertion_data_port;
	lib::smb_multi_pin_insertion smb_multi_pin_insertion_structure;

	lib::single_thread_request_port<lib::smb_multi_leg_reply>
			* smb_multi_leg_reply_data_request_port;
	lib::smb_multi_leg_reply edp_ecp_smb_multi_leg_reply_structure;

public:
	pin_rise(common::task::task& _ecp_task); //constructor
	bool first_step(); //first step generation
	bool next_step(); //next step generation

	void create_ecp_mp_reply();
	void get_mp_ecp_command();

};

class pin_lower: public common::generator::generator {
private:
	lib::smb_multi_pin_insertion mp_ecp_smb_multi_pin_insertion_structure;

	lib::single_thread_port<lib::smb_multi_pin_insertion>
			* smb_multi_pin_insertion_data_port;
	lib::smb_multi_pin_insertion smb_multi_pin_insertion_structure;

	lib::single_thread_request_port<lib::smb_multi_leg_reply>
			* smb_multi_leg_reply_data_request_port;
	lib::smb_multi_leg_reply edp_ecp_smb_multi_leg_reply_structure;
public:
	pin_lower(common::task::task& _ecp_task); //constructor
	bool first_step(); //first step generation
	bool next_step(); //next step generation

	void create_ecp_mp_reply();
	void get_mp_ecp_command();

};

} // namespace generator
} // namespace smb
} // namespace ecp
} // namespace mrrocpp

#endif /* ECP_G_SLEEP_H_ */
