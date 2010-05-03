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

public:
	pin_lock(common::task::task& _ecp_task); //constructor
	bool first_step(); //first step generation
	bool next_step(); //next step generation

	void create_ecp_mp_reply();
	void get_mp_ecp_command();

};

class pin_unlock: public common::generator::generator {
private:

public:
	pin_unlock(common::task::task& _ecp_task); //constructor
	bool first_step(); //first step generation
	bool next_step(); //next step generation

	void create_ecp_mp_reply();
	void get_mp_ecp_command();

};

class pin_rise: public common::generator::generator {
private:

public:
	pin_rise(common::task::task& _ecp_task); //constructor
	bool first_step(); //first step generation
	bool next_step(); //next step generation

	void create_ecp_mp_reply();
	void get_mp_ecp_command();

};

class pin_lower: public common::generator::generator {
private:

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
