/*!
 * @file mp_t_single_agent_demo.cpp
 *
 * @date Jan 20, 2012
 * @author tkornuta
 */

#include <fstream>

#include <boost/foreach.hpp>

#include "mp_t_single_agent_plan.h"

#include "base/lib/configurator.h"
#include "base/ecp_mp/ecp_ui_msg.h"

#include "../swarmitfix_plan/plan_iface.h"
#include "plan.hxx"

namespace mrrocpp {
namespace mp {
namespace task {

task* return_created_mp_task(lib::configurator &_config)
{
	return new swarmitfix::single_agent_demo(_config);
}

namespace swarmitfix {

void single_agent_demo::executeCommandItem(const Plan::PkmType::ItemType & pkmCmd)
{
	// Make sure that there are only Xyz-Euler-Zyz coordinates.
	assert(pkmCmd.pkmToWrist().present() == false);

	// Setup command for the PKM
	lib::epos::EPOS_MOTION_VARIANT motion_variant;

	// Goal pose
	lib::Xyz_Euler_Zyz_vector goal(
			pkmCmd.Xyz_Euler_Zyz()->x(),
			pkmCmd.Xyz_Euler_Zyz()->y(),
			pkmCmd.Xyz_Euler_Zyz()->z(),
			pkmCmd.Xyz_Euler_Zyz()->alpha(),
			pkmCmd.Xyz_Euler_Zyz()->beta(),
			pkmCmd.Xyz_Euler_Zyz()->gamma()
			);

	std::cout << "Xyz_Euler_Zyz: " <<
			pkmCmd.Xyz_Euler_Zyz()->x() << " " <<
			pkmCmd.Xyz_Euler_Zyz()->y() << " " <<
			pkmCmd.Xyz_Euler_Zyz()->z() << " " <<
			pkmCmd.Xyz_Euler_Zyz()->alpha() << " " <<
			pkmCmd.Xyz_Euler_Zyz()->beta() << " " <<
			pkmCmd.Xyz_Euler_Zyz()->gamma() << std::endl;

	switch(pkmCmd.ind() % 100) {
		case 0:
		case 20:
		case 40:
		case 60:
		case 80:
			motion_variant = lib::epos::SYNC_TRAPEZOIDAL;
			break;
		default:
			throw std::runtime_error("Unexpected 'ind' in PKM plan item");
			break;
	}

	// Check if robot name match with item.
	if((spkm_robot_name == lib::spkm1::ROBOT_NAME && pkmCmd.agent() == 1) ||
		(spkm_robot_name == lib::spkm2::ROBOT_NAME && pkmCmd.agent() == 2)) {
		// Execute command.
		move_spkm_external(motion_variant, goal);
		//move_shead_joints(pkmCmd.beta7());
	}
}

void single_agent_demo::executeCommandItem(const Plan::MbaseType::ItemType & smbCmd)
{
	// TODO: Only single-item actions are supported at this time.
	assert(smbCmd.actions().item().size() == 1);

	// Check if robot name match with item.
	if((smb_robot_name == lib::smb1::ROBOT_NAME && smbCmd.agent() == 1) ||
		(smb_robot_name == lib::smb2::ROBOT_NAME && smbCmd.agent() == 2)) {
		// Execute command.
		smb_rotate_external(0, smbCmd.actions().item().front().dPkmTheta());
		//move_shead_joints(pkmCmd.beta7());
	}
}

single_agent_demo::single_agent_demo(lib::configurator &config_) :
		demo_base(config_)
{
	// Read plan from file.
	p = readPlanFromFile(config.value<std::string>("planpath"));

	// SMB.
	if (IS_MP_ROBOT_ACTIVE (smb1)) {
		smb_robot_name = lib::smb1::ROBOT_NAME;
	} else if (IS_MP_ROBOT_ACTIVE (smb2)) {
		smb_robot_name = lib::smb2::ROBOT_NAME;
	} else {
		// TODO: throw - but what??
	}
	// SPKM.
	if (IS_MP_ROBOT_ACTIVE (spkm1)) {
		spkm_robot_name = lib::spkm1::ROBOT_NAME;
	} else if (IS_MP_ROBOT_ACTIVE (spkm2)) {
		spkm_robot_name = lib::spkm2::ROBOT_NAME;
	} else {
		// TODO: throw - but what??
	}
	// SHEAD.
	if (IS_MP_ROBOT_ACTIVE (shead1)) {
		shead_robot_name = lib::shead1::ROBOT_NAME;
	} else if (IS_MP_ROBOT_ACTIVE (smb2)) {
		shead_robot_name = lib::shead2::ROBOT_NAME;
	} else {
		// TODO: throw - but what??
	}

}

void single_agent_demo::create_robots()
{
	// Activate robots (depending on the configuration settings).
	// SMB.
	if (smb_robot_name == lib::smb1::ROBOT_NAME) {
		ACTIVATE_MP_ROBOT(smb1)
	} else {
		ACTIVATE_MP_ROBOT(smb2)
	}
	// SPKM.
	if (spkm_robot_name == lib::spkm1::ROBOT_NAME) {
		ACTIVATE_MP_ROBOT(spkm1)
	} else {
		ACTIVATE_MP_ROBOT(spkm2)
	}
	// SHEAD.
	if (shead_robot_name == lib::shead1::ROBOT_NAME) {
		ACTIVATE_MP_ROBOT(shead1)
	} else {
		ACTIVATE_MP_ROBOT(shead2)
	}

	// Activate the SBENCH robot.
	ACTIVATE_MP_ROBOT(sbench)
}

void single_agent_demo::save_plan(const Plan & p)
{
	std::cout << "Save to file." << std::endl;
	std::string savepath = "foo.xml"; // config.value<std::string>(planner::planpath);
	std::ofstream ofs (savepath.c_str());
	plan(ofs, p);
}

lib::UI_TO_ECP_REPLY single_agent_demo::step_mode(Mbase::ItemType & item)
{
	// Create the text representation.
	std::ostringstream ostr;
	{
		boost::archive::text_oarchive oa(ostr);
		xml_schema::ostream<boost::archive::text_oarchive> os(oa);

		// serialize data
		os << item;
	}

	// Request
	lib::ECP_message ecp_to_ui_msg;

	// Setup plan item
	ecp_to_ui_msg.ecp_message = lib::PLAN_STEP_MODE;
	ecp_to_ui_msg.plan_item_type = lib::MBASE_AND_BENCH;
	ecp_to_ui_msg.plan_item_string = ostr.str();

	// Reply
	lib::UI_reply ui_to_ecp_rep;

	if (messip::port_send(UI_fd, 0, 0, ecp_to_ui_msg, ui_to_ecp_rep) < 0) {

		uint64_t e = errno;
		perror("ecp operator_reaction(): Send() to UI failed");
		sr_ecp_msg->message(lib::SYSTEM_ERROR, e, "ecp: Send() to UI failed");
		BOOST_THROW_EXCEPTION(lib::exception::system_error());
	}

	if(ui_to_ecp_rep.reply == lib::PLAN_EXEC) {
		std::istringstream istr(ui_to_ecp_rep.plan_item_string);
		boost::archive::text_iarchive ia(istr);
		xml_schema::istream<boost::archive::text_iarchive> is (ia);

		// deserialize data
		item = Mbase::ItemType(is);
	}

	return ui_to_ecp_rep.reply;
}

lib::UI_TO_ECP_REPLY single_agent_demo::step_mode(Pkm::ItemType & item)
{
	// create archive
	std::ostringstream ostr;
	{
		boost::archive::text_oarchive oa(ostr);
		xml_schema::ostream<boost::archive::text_oarchive> os(oa);

		// serialize data
		os << item;
	}

	// Request
	lib::ECP_message ecp_to_ui_msg;

	// Setup plan item
	ecp_to_ui_msg.ecp_message = lib::PLAN_STEP_MODE;
	ecp_to_ui_msg.plan_item_type = lib::PKM_AND_HEAD;
	ecp_to_ui_msg.plan_item_string = ostr.str();

	// Reply
	lib::UI_reply ui_to_ecp_rep;

	if (messip::port_send(UI_fd, 0, 0, ecp_to_ui_msg, ui_to_ecp_rep) < 0) {

		uint64_t e = errno;
		perror("ecp operator_reaction(): Send() to UI failed");
		sr_ecp_msg->message(lib::SYSTEM_ERROR, e, "ecp: Send() to UI failed");
		BOOST_THROW_EXCEPTION(lib::exception::system_error());
	}

	if(ui_to_ecp_rep.reply == lib::PLAN_EXEC) {
		std::istringstream istr(ui_to_ecp_rep.plan_item_string);
		boost::archive::text_iarchive ia(istr);
		xml_schema::istream<boost::archive::text_iarchive> is (ia);

		// deserialize data
		item = Pkm::ItemType(is);
	}

	return ui_to_ecp_rep.reply;
}

void single_agent_demo::main_task_algorithm(void)
{
	/////////////////////////////////////////////////////////////////////////////////
	using namespace mrrocpp::lib::sbench;
	/////////////////////////////////////////////////////////////////////////////////

	// Time index counter
	int indMin = 0, indMax = 0;

	// Setup index counter at the beginning of the plan
	BOOST_FOREACH(const Plan::PkmType::ItemType & it, p->pkm().item()) {
		if(indMin > it.ind()) indMin = it.ind();
		if(indMax < it.ind()) indMax = it.ind();
	}
	BOOST_FOREACH(const Plan::MbaseType::ItemType & it, p->mbase().item()) {
		if(indMin > it.ind()) indMin = it.ind();
		if(indMax < it.ind()) indMax = it.ind();
	}

	for (int ind = indMin, dir = 0; true; ind += dir) {

		if(ind < indMin) ind = indMin;
		if(ind > indMax) ind = indMax;

		std::cout << "plan index = " << ind << std::endl;

		// Diagnostic timestamp
		boost::system_time start_timestamp;

		// Plan iterators
		const Plan::PkmType::ItemSequence::iterator pkm_it = StateAtInd(ind, p->pkm().item());
		const Plan::MbaseType::ItemSequence::iterator smb_it = StateAtInd(ind, p->mbase().item());

		// Current state
		State * currentActionState;

		bool record_timestamp = false;

		// Execute matching command item
		if(pkm_it != p->pkm().item().end()) {

			switch(step_mode(*pkm_it)) {
				case lib::PLAN_PREV:
					dir = -1;
					break;
				case lib::PLAN_NEXT:
					dir = +1;
					break;
				case lib::PLAN_EXEC:
					currentActionState = (State *) &(*pkm_it);
					start_timestamp = boost::get_system_time();
					executeCommandItem(*pkm_it);
					record_timestamp = true;
					dir = 0;
					break;
				case lib::PLAN_SAVE:
					save_plan(*p);
					dir = 0;
					break;
				default:
					break;
			}
		} else if(smb_it != p->mbase().item().end()) {

			switch(step_mode(*smb_it)) {
				case lib::PLAN_PREV:
					dir = -1;
					break;
				case lib::PLAN_NEXT:
					dir = +1;
					break;
				case lib::PLAN_EXEC:
					currentActionState = (State *) &(*smb_it);
					start_timestamp = boost::get_system_time();
					executeCommandItem(*smb_it);
					record_timestamp = true;
					dir = 0;
					break;
				case lib::PLAN_SAVE:
					save_plan(*p);
					dir = 0;
					break;
				default:
					break;
			}
		} else {
			continue;
		}


		// Diagnostic timestamp
		if (record_timestamp) {
			// Get the stop timestamp
			boost::system_time stop_timestamp = boost::get_system_time();

			// Calculate command duration
			boost::posix_time::time_duration td = stop_timestamp - start_timestamp;

			std::cout << "Command duration in [ms] is " << td.total_milliseconds() << std::endl;
			currentActionState->state_reached_in_time().set(td.total_milliseconds()/1000.0);

//			// Wait for trigger
//			while(!(ui_pulse.isFresh() && ui_pulse.Get() == MP_TRIGGER)) {
//				// TODO: handle PAUSE/RESUME/STOP commands as well
//				if(ui_pulse.isFresh()) ui_pulse.markAsUsed();
//				ReceiveSingleMessage(true);
//			}
//
//			ui_pulse.markAsUsed();

		}

		// If all iterators are at the end...
		if(ind == indMax && dir > 0)
		{
			// ...then finish
			break;
		}
	}

	// Serialize to a file.
	//
	{
		std::cout << "Serialize to a file." << std::endl;
		std::ofstream ofs ("result.xml");
		plan(ofs, *p);
	}

	sr_ecp_msg->message("END");


	/////////////////////////////////////////////////////////////////////////////////
	/////////////////////////////////////////////////////////////////////////////////
	/////////////////////////////////////////////////////////////////////////////////
	/////////////////////////////////////////////////////////////////////////////////
	/////////////////////////////////////////////////////////////////////////////////

	return;

	/////////////////////////////////////////////////////////////////////////////////


	sr_ecp_msg->message("smb_powered_from_bench_test::main_task_algorithm");
	int mode = config.value <int>("mode");
	unsigned int delay = config.value <unsigned int>("delay");
	//unsigned int cleaning_time = config.value <int>("cleaning_time");

	// 1st pose.
	bench_pose pose1;
	pose1.pins[0] = pin(4, 4);
	pose1.pins[1] = pin(3, 3);
	pose1.pins[2] = pin(4, 3);

	// 2nd pose.
	bench_pose pose2;
	pose2.pins[0] = pin(4, 4);
	pose2.pins[1] = pin(4, 3);
	pose2.pins[2] = pin(5, 3);

	// 3rd pose.
	bench_pose pose3;
	pose3.pins[0] = pin(6, 3);
	pose3.pins[1] = pin(6, 4);
	pose3.pins[2] = pin(5, 3);

	// Power trajectory.
	power_smb_move move1(pose1, pose2, pkm_leg_rotation(1, 1));
	power_smb_move move2(pose2, pose3, pkm_leg_rotation(3, 3));
	power_smb_move move3(pose3, pose2, pkm_leg_rotation(3, -3));
	power_smb_move move4(pose2, pose1, pkm_leg_rotation(1, -1));

	// Support poses.
	/*
	 neutral pose OK
	 tool:  -0.1412 -0.035 0.5768 3.1416 0.137 0
	 wrist: 0.15 -0.035 0.405 0 -0.92 0
	 */
	lib::Xyz_Euler_Zyz_vector neutral_pose(0.15, -0.035, 0.405, 0, -0.92, 0);

	/*
	 Podparcie nr 1: smb rot = 1
	 tool:  -0.33 0.0012 0.625 -0.8193 0.029 -2.3437
	 wrist: -0.0693 0 0.4097 0 -0.763 -0.03

	 odejście:
	 tool:  -0.33 0 0.59 3.1416 0.02 0
	 wrist: -0.0609 0 0.3853 0 -0.803 0
	 */
	lib::Xyz_Euler_Zyz_vector support_pose1(-0.0693, 0, 0.4097, 0, -0.763, -0.03);
	lib::Xyz_Euler_Zyz_vector inter_pose1(-0.0609, 0, 0.3853, 0, -0.803, 0);

	/*
	 Podparcie 2: smb rot = 0, legs out
	 tool:   -0.2919 0 0.626 -3.1416 -0.03 0
	 wrist:  -0.0333 0 0.4081 0 -0.753 0

	 odejście z podparcia nr 1:
	 tool:  -0.2919 0 0.6 -3.1416 0 0
	 wrist: -0.0269 0 0.39 0 -0.783 0
	 */
	lib::Xyz_Euler_Zyz_vector support_pose2(-0.0333, 0, 0.4081, 0, -0.753, 0);
	lib::Xyz_Euler_Zyz_vector inter_pose2(-0.0269, 0, 0.39, 0, -0.783, 0);

	/*
	 Podparcie nr 3: smb rot = -1
	 tool: -0.3691 0 0.627 3.1416 -0.05 0
	 wrist: -0.1149 0 0.404 0 -0.733 0

	 odejście:
	 tool:  -0.3691 0 0.5847 3.1416 0.02 0
	 wrist: -0.1 0 0.38 0 -0.803 0
	 */
	lib::Xyz_Euler_Zyz_vector support_pose3(-0.1149, 0, 0.404, 0, -0.733, 0);
	lib::Xyz_Euler_Zyz_vector inter_pose3(-0.1, 0, 0.38, 0, -0.803, 0);

	// Pull out all legs.
//	smb_pull_legs(lib::smb::OUT, lib::smb::OUT, lib::smb::OUT);
	// Wait for given time.
//	wait_ms(delay);

	// Turn power on 1st pose pins.
	sr_ecp_msg->message(pose1.get_description());
	mrrocpp::lib::sbench::power_supply_state ps;
	ps.set_on(pose1);
	control_bench_power_supply(ps, delay);

	if ((mode == 0) || (mode == 2)) {
		// Move to the *neutral* PKM pose.
		move_spkm_external(lib::epos::SYNC_TRAPEZOIDAL, neutral_pose);

		// Move shead to synchro position.
		move_shead_joints(0.0);

		// Move to first pose.
		move_to_pose_and_return(support_pose1, inter_pose1, 1, 0.5);
	}

	if ((mode == 1) || (mode == 2)) {
		smb_execute_power_move(move1, delay);
	}

	if ((mode == 0) || (mode == 2)) {
		move_to_pose_and_return(support_pose2, inter_pose2, 0, -0.5);
		move_to_pose_and_return(support_pose3, inter_pose3, -1, 0);
	}

	if ((mode == 1) || (mode == 2)) {
		smb_execute_power_move(move2, delay);
	}
	if ((mode == 0) || (mode == 2)) {
		move_to_pose_and_return(support_pose2, inter_pose2, 0, -0.5);
		move_to_pose_and_return(support_pose1, inter_pose1, 1, 0.5);
	}
	if ((mode == 1) || (mode == 2)) {
		smb_execute_power_move(move3, delay);
	}
	if ((mode == 0) || (mode == 2)) {
		move_to_pose_and_return(support_pose3, inter_pose3, -1, 0);
	}
	if ((mode == 1) || (mode == 2)) {
		smb_execute_power_move(move4, delay);
	}
	sr_ecp_msg->message("Task finished");

}

} /* namespace swarmitfix */
} /* namespace task */
} /* namespace mp */
} /* namespace mrrocpp */
