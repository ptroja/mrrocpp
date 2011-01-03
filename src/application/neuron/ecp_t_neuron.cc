/**
 * @file mp_t_neuron.cc
 * @brief Source file for Neuron task class.
 * @author Tomasz Bem (mebmot@wp.pl)
 * @ingroup neuron
 * @date 13.05.2010
 */

#include "robot/irp6ot_m/ecp_r_irp6ot_m.h"
#include "ecp_t_neuron.h"
#include "neuron_sensor.h"
#include "ecp_mp_t_neuron.h"
#include "ecp_mp_neuron_sensor.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot {
namespace task {

/*==============================Constructor===============================*//**
 * @brief Constructor along with appropriate configuration.
 * @param _config Configuration object reference.
 */
Neuron::Neuron(lib::configurator &_config): task(_config){
	ecp_m_robot=new irp6ot_m::robot(*this);				//initialization of robot
	smoothGenerator=new common::generator::newsmooth(*this, lib::ECP_XYZ_ANGLE_AXIS, 6);
	sr_ecp_msg->message("ECP loaded Neuron");
};

/*============================Destructor==================================*//**
 * @brief Destructor.
 * @details Cleans it own shit.
 */
Neuron::~Neuron(){
	delete ecp_m_robot;
	delete neuronGenerator;
	delete smoothGenerator;
};

/*====================mp_2_ecp_next_state_string_handler==================*//**
 * @brief Method called from main_task_algorithm to handle next_state command.
 * @details Method that handles main algorithm and informatin flow for Neuron
 * task. Starts generators and waits for start and stop signal from VSP.
 */
void Neuron::mp_2_ecp_next_state_string_handler(void){
	sr_ecp_msg->message("poczatek");
	if (mp_2_ecp_next_state_string == ecp_mp::task::ECP_T_NEURON) {
		//Initalizing all needed items: sensors and generators.
		neuronSensor = new ecp_mp::sensor::neuron_sensor(config);
		sensor_m[ecp_mp::sensor::ECP_MP_NEURON_SENSOR] = neuronSensor;
		neuronGenerator = new common::generator::neuron_generator(*this);
		neuronGenerator->sensor_m=sensor_m;

		//coordinates sent from VSP to MRROC++;
		std::vector<double> coordinates1(6);
		ecp_mp::sensor::Coordinates coordinates;

		//loop that enables starting and stopping work of MRROC++ from VSP.
		while(true){
			sr_ecp_msg->message("czekam na start");
			//Waits for Start command from VSP.
			neuronSensor->waitForVSPStart();

			//loop that executes one trajectory currently managed by VSP at
			//one time. Loop loops until VSP has another trajectory to process.
			while(true){
				//Obtain first coordinates from VSP for smooth generator.
				coordinates=neuronSensor->getFirstCoordinates();

				smoothGenerator->reset();
				smoothGenerator->set_absolute();

				//set coordinates for smooth generator.
				coordinates1[0]=coordinates.x;
				coordinates1[1]=coordinates.y;
				coordinates1[2]=coordinates.z;
				coordinates1[3]=1.203;
				coordinates1[4]=-1.447;
				coordinates1[5]=-0.294;
				smoothGenerator->load_absolute_angle_axis_trajectory_pose(coordinates1);

				//Interpolate trajectory and run generator.
				smoothGenerator->set_debug(false);
				if(smoothGenerator->calculate_interpolate())
					smoothGenerator->Move();

				//starts Neuron generator.
				neuronGenerator->reset();
				neuronGenerator->Move();

				//If Stop button in VSP was pressed, stop processing
				//more trajectories and wait at the beginning of first
				//loop for another start signal.
				if(neuronSensor->stop())
					break;

				double * finalPosition;
				finalPosition=neuronGenerator->get_position();
				neuronSensor->sendFinalPosition(finalPosition[0], finalPosition[1], finalPosition[2]);
			}
		}

		//Inform VSP that the communication link should be closed.
		neuronSensor->sendCommunicationFinished();
	}
	sr_ecp_msg->message("koniec");
}

/*==========================ecp_stop_accepted_handler=====================*//**
 * @brief Handler for stop button in MRROC++ taks panel.
 * @details After pressing stop button in MRROC++ it sends message to VSP, that
 * MRROC++ is no longer available for work therefore neuron sensor is deleted
 * from the map of sensors.
 */
void Neuron::ecp_stop_accepted_handler(){
	sr_ecp_msg->message("mp_stop_pressed");
	neuronSensor->sendCommunicationFinished();
	delete sensor_m[ecp_mp::sensor::ECP_MP_NEURON_SENSOR];
	sensor_m.erase(ecp_mp::sensor::ECP_MP_NEURON_SENSOR);
}

}  //namespace task
} // namespace irp6ot

namespace common {
namespace task {

/*============================return_created_ecp_taks=====================*//**
* @brief returns inherited task pointer.
* @param _config configurator object reference.
* @return inherited task pointer.
*/
task* return_created_ecp_task(lib::configurator &_config){
	return new irp6ot::task::Neuron(_config);
}

} // namespace task
} // namespace common
} // namespace ecp
} // namespace mrrocpp

