/*
 * ecp_g_discode_sensor_test.cc
 *
 *  Created on: Nov 4, 2010
 *      Author: mboryn
 */

#include <cstdio>
#include <string>
#include "ecp_g_discode_sensor_test.h"
#include "base/lib/mrmath/mrmath.h"
#include "base/ecp/ecp_robot.h"
#include "base/lib/logger.h"

namespace mrrocpp {

namespace ecp {

namespace common {

namespace generator {

using boost::shared_ptr;
using namespace std;
using namespace logger;

ecp_g_discode_sensor_test::ecp_g_discode_sensor_test(mrrocpp::ecp::common::task::task & ecp_task, mrrocpp::ecp_mp::sensor::discode::discode_sensor *ds) :
	generator(ecp_task), ds(ds)
{
	// TODO Auto-generated constructor stub
	sensor_m["my_discode_sensor"] = ds;
}

ecp_g_discode_sensor_test::~ecp_g_discode_sensor_test()
{
	// TODO Auto-generated destructor stub
}

bool ecp_g_discode_sensor_test::first_step()
{
	int motion_steps = 30;
	the_robot->ecp_command.instruction.instruction_type = lib::GET;
	the_robot->ecp_command.instruction.get_type = ARM_DEFINITION;
	the_robot->ecp_command.instruction.get_arm_type = lib::FRAME;
	the_robot->ecp_command.instruction.motion_type = lib::ABSOLUTE;
	the_robot->ecp_command.instruction.set_type = ARM_DEFINITION;
	the_robot->ecp_command.instruction.set_arm_type = lib::FRAME;
	the_robot->ecp_command.instruction.interpolation_type = lib::TCIM;
	the_robot->ecp_command.instruction.motion_steps = motion_steps;
	the_robot->ecp_command.instruction.value_in_step_no = motion_steps - 3;

	for (int i = 0; i < 6; i++) {
		the_robot->ecp_command.instruction.arm.pf_def.behaviour[i] = lib::UNGUARDED_MOTION;
	}

	jjj = 0;

	return true;
}

bool ecp_g_discode_sensor_test::next_step()
{
	the_robot->ecp_command.instruction.instruction_type = lib::SET_GET;
	lib::Homog_matrix current_position;
	current_position.set_from_frame_tab(the_robot->reply_package.arm.pf_def.arm_frame);
	current_position.get_frame_tab(the_robot->ecp_command.instruction.arm.pf_def.arm_frame);

//	log_dbg("bool ecp_g_discode_sensor_test::next_step()\n");

	//	shared_ptr <xdr_iarchive <> > ia = ds->get_iarchive();
	//	string s;
	//	*ia >> s;
	//	log("ecp_g_discode_sensor_test::next_step() received: \"%s\"", s.c_str());
	//
	//	*(ds->get_oarchive()) << string("bool ecp_g_discode_sensor_test::next_step()");

	//	jjj++;
	//	if (jjj % 4 == 0) {
	//		*ds->get_oarchive() << jjj;
	//	}
	//	if (jjj % 20 == 0) {
	//		ds.get_oarchive()->clear_buffer();
	//	}
	if (ds->is_reading_ready()) {
		string s = ds->get_received_object <string> ();
		log("ecp_g_discode_sensor_test::next_step(): \"%s\"\n", s.c_str());
	} else {
		log("ecp_g_discode_sensor_test::next_step(): reading not ready.\n");
	}
	return true;
}

} // namespace generator

} // namespace common

} // namespace ecp

} // namespace mrrocpp
