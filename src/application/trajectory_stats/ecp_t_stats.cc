/**
 * @file mp_t_stats.cc
 * @brief Source file for Stats task class.
 * @author Tomasz Bem (mebmot@wp.pl)
 * @ingroup stats
 * @date 13.05.2010
 */

#include "robot/irp6p_m/ecp_r_irp6p_m.h"
#include "ecp_t_stats.h"
#include "ecp_mp_t_stats.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace task {

/*==============================Constructor===============================*//**
 * @brief Constructor along with appropriate configuration.
 * @param _config Configuration object reference.
 */
Stats::Stats(lib::configurator &_config) :
	common::task::task(_config)
{
	ecp_m_robot = (boost::shared_ptr <robot_t>) new irp6p_m::robot(*this); //initialization of robot
	smoothGenerator = new common::generator::newsmooth(*this, lib::ECP_XYZ_ANGLE_AXIS, 6);
	sr_ecp_msg->message("ECP loaded Stats");
}

/*============================Destructor==================================*//**
 * @brief Destructor.
 * @details Cleans it own shit.
 */
Stats::~Stats()
{
	delete statsGenerator;
	delete smoothGenerator;
}

/*====================mp_2_ecp_next_state_string_handler==================*//**
 * @brief Method called from main_task_algorithm to handle next_state command.
 * @details Method that handles main algorithm and information flow for Stats
 * task. Starts generators and waits for start and stop signal from VSP.
 */
void Stats::mp_2_ecp_next_state_string_handler(void)
{
	sr_ecp_msg->message("poczatek");
	if (mp_2_ecp_next_state_string == ecp_mp::task::ECP_T_STATS) {


		//Initalizing all needed items: sensors and generators.
		statsGenerator = new common::generator::stats_generator(*this);

		//starts stats generator.
		statsGenerator->reset();
		statsGenerator->Move();
	}
	sr_ecp_msg->message("koniec");
}

/*==========================ecp_stop_accepted_handler=====================*//**
 * @brief Handler for stop button in MRROC++ task panel.
 * @details After pressing stop button in MRROC++ it sends message to VSP, that
 * MRROC++ is no longer available for work therefore Stats sensor is deleted
 * from the map of sensors.
 */
void Stats::ecp_stop_accepted_handler()
{
	sr_ecp_msg->message("ecp_stop_pressed");
}

} //namespace task
} // namespace irp6ot

namespace common {
namespace task {

/*============================return_created_ecp_taks=====================*//**
 * @brief returns inherited task pointer.
 * @param _config configurator object reference.
 * @return inherited task pointer.
 */
task_base* return_created_ecp_task(lib::configurator &_config)
{
	return new common::task::Stats(_config);
}

} // namespace task
} // namespace common
} // namespace ecp
} // namespace mrrocpp

