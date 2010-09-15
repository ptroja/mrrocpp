/*
 * generator/ecp_g_head.h
 *
 *Author: yoyek
 */

#ifndef ECP_G_SHEAD_H_
#define ECP_G_SHEAD_H_

#include "base/ecp/ecp_generator.h"
#include "robot/shead/dp_shead.h"

namespace mrrocpp {
namespace ecp {
namespace shead {
namespace generator {

class head_soldify : public common::generator::generator
{
private:
	lib::shead::HEAD_SOLIDIFICATION mp_ecp_shead_head_soldification_structure;

	lib::single_thread_port <lib::shead::HEAD_SOLIDIFICATION> * shead_head_soldification_data_port;

	lib::single_thread_request_port <lib::shead::reply> * shead_reply_data_request_port;

public:
	head_soldify(common::task::task& _ecp_task); //constructor
	bool first_step(); //first step generation
	bool next_step(); //next step generation

	void create_ecp_mp_reply();
	void get_mp_ecp_command();

};

class head_desoldify : public common::generator::generator
{
private:
	lib::shead::HEAD_SOLIDIFICATION mp_ecp_shead_head_soldification_structure;

	lib::single_thread_port <lib::shead::HEAD_SOLIDIFICATION> * shead_head_soldification_data_port;
	lib::shead::HEAD_SOLIDIFICATION ecp_edp_shead_head_soldification_structure;

	lib::single_thread_request_port <lib::shead::reply> * shead_reply_data_request_port;
	lib::shead::reply ecp_edp_shead_reply_structure;

public:
	head_desoldify(common::task::task& _ecp_task); //constructor
	bool first_step(); //first step generation
	bool next_step(); //next step generation

	void create_ecp_mp_reply();
	void get_mp_ecp_command();

};

class head_vacuum_on : public common::generator::generator
{
private:
	lib::shead::VACUUM_ACTIVATION mp_ecp_shead_vacuum_activation_structure;

	lib::single_thread_port <lib::shead::VACUUM_ACTIVATION> * shead_vacuum_activation_data_port;
	lib::shead::VACUUM_ACTIVATION ecp_edp_shead_vacuum_activation_structure;

	lib::single_thread_request_port <lib::shead::reply> * shead_reply_data_request_port;
	lib::shead::reply ecp_edp_shead_reply_structure;

public:
	head_vacuum_on(common::task::task& _ecp_task); //constructor
	bool first_step(); //first step generation
	bool next_step(); //next step generation

	void create_ecp_mp_reply();
	void get_mp_ecp_command();
};

class head_vacuum_off : public common::generator::generator
{
private:
	lib::shead::VACUUM_ACTIVATION mp_ecp_shead_vacuum_activation_structure;

	lib::single_thread_port <lib::shead::VACUUM_ACTIVATION> * shead_vacuum_activation_data_port;
	lib::shead::VACUUM_ACTIVATION ecp_edp_shead_vacuum_activation_structure;

	lib::single_thread_request_port <lib::shead::reply> * shead_reply_data_request_port;
	lib::shead::reply ecp_edp_shead_reply_structure;

public:
	head_vacuum_off(common::task::task& _ecp_task); //constructor
	bool first_step(); //first step generation
	bool next_step(); //next step generation

	void create_ecp_mp_reply();
	void get_mp_ecp_command();

};

} // namespace generator
} // namespace shead
} // namespace ecp
} // namespace mrrocpp

#endif /* ECP_G_SLEEP_H_ */
