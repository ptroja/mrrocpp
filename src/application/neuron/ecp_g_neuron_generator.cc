/**
 * @file ecp_g_neuron_generator.cc
 * @brief Header file for neuron_generator class
 * @author Tomasz Bem (mebmot@wp.pl)
 * @author Rafal Tulwin (rtulwin@gmail.com)
 * @ingroup neuron
 * @date 02.07.2010
 */

#include "base/ecp/ecp_robot.h"
#include "base/ecp/ecp_task.h"

#include "ecp_g_neuron_generator.h"
#include "ecp_mp_neuron_sensor.h"
#include <ctime>

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

const double MIN_VELOCITY = 0.1;
const double MSTEP_TIME = 0.002 * 10.0;


static inline void generatePowers(int n, double x, double* powers)
{
  powers[0] = 1.0;
  for (int i=1; i<=n; i++)
  {
    powers[i] = powers[i-1]*x;
  }
}

/*================================Constructor=============================*//**
 * @brief Constructor along with task configurator.
 * @param _ecp_task Reference to task configurator.
 */
neuron_generator::neuron_generator(common::task::task& _ecp_task) :
	common::generator::generator(_ecp_task)
{
	reset();
}

/*==============================Destructor================================*//**
 * @brief Destructor.
 */
neuron_generator::~neuron_generator()
{
}

/*===============================first_step===============================*//**
 * @brief First step of neuron generator
 * @details Initializes instruction for robot and sends information to VSP that
 * new trajectory execution is starting, therefore requests first coordinates
 * of a trajectory.
 */
bool neuron_generator::first_step()
{
	sr_ecp_msg.message("neuron generator first step");
	printf("neuron generator first step\n");
	the_robot->ecp_command.instruction_type = lib::GET;
	the_robot->ecp_command.get_type = ARM_DEFINITION;
	the_robot->ecp_command.set_type = ARM_DEFINITION;
	the_robot->ecp_command.set_arm_type = lib::FRAME;
	the_robot->ecp_command.get_arm_type = lib::FRAME;
	the_robot->ecp_command.interpolation_type = lib::MIM;
	the_robot->ecp_command.motion_steps = 10;
	the_robot->ecp_command.value_in_step_no = 10 - 2;
	the_robot->ecp_command.motion_type = lib::ABSOLUTE;

	//get neuron sensor and send information about starting new trajectory.
	neuron_sensor = (ecp_mp::sensor::neuron_sensor*) sensor_m[ecp_mp::sensor::ECP_MP_NEURON_SENSOR];
	neuron_sensor->startGettingTrajectory();
	macroSteps=neuron_sensor->getMacroStepsNumber();
	radius=neuron_sensor->getRadius();
	neuron_sensor->get_reading();
	printf("macroStep: %d\n",macroSteps);
	printf("radius: %f\n",radius);

	mstep_ = 0;

	return true;
}

/*================================next_step===============================*//**
 * @brief next step for neuron generator.
 * @details Next step interpolates 5 consecutive macro steps basing on received
 * coordinates from VSP. It also receives information whether to start breaking
 * or not. More over it calculates the overshoot of the manipulator.
 */
bool neuron_generator::next_step()
{
	the_robot->ecp_command.instruction_type = lib::SET;
	flushall();

	//Check if stop button in VSP was pressed
	if (neuron_sensor->stop()) {
		printf("VSP Stop button pressed\n");
		return false;
	}

	//when current period == 5 then new data from VSP is ready, therefore
	//generator has to interpolate this coordinates for 5 macro steps.
	//this section is not performed in breaking phase, during which in every
	//next step period is set to 4
	if (neuron_sensor->newData() && !breaking_) {
		printf("\n-------- new interpolation point ----------\n");
		flushall();
		// ------------ read the current robot position ----------
		actual_position_matrix.set_from_frame_tab(the_robot->reply_package.arm.pf_def.arm_frame);
		actual_position_matrix.get_xyz_angle_axis(angle_axis_vector);
		angle_axis_vector.to_table(actual_position);
		// ------------ read the current robot position (end) ---------

		//get desired position for robot.
		flushall();
		desired_position[0] = neuron_sensor->getCoordinates().x;
		desired_position[1] = neuron_sensor->getCoordinates().y;
		desired_position[2] = neuron_sensor->getCoordinates().z;
		desired_position[3] = position[3] = actual_position[3];
		desired_position[4] = position[4] = actual_position[4];
		desired_position[5] = position[5] = actual_position[5];

		normalized_vector[0] = neuron_sensor->getLastButOne().x;
		normalized_vector[1] = neuron_sensor->getLastButOne().y;
		normalized_vector[2] = neuron_sensor->getLastButOne().z;

        if(neuron_sensor->startBraking()) {
        	double time;
        	neuron_sensor->stopReceivingData();
			breaking_ = true;

			printf("\n-------- breaking ----------\n");
			flushall();

            if( sqrt(vel_[0]*vel_[0] + vel_[1]*vel_[1] + vel_[2]*vel_[2]) < MIN_VELOCITY)
			{
				time = 2 * MIN_VELOCITY / radius;
			} else {
                                time = 2 * sqrt(vel_[0]*vel_[0] + vel_[1]*vel_[1] + vel_[2]*vel_[2]) / radius;
			}

			break_steps_ = time / ((double)macroSteps * MSTEP_TIME);

			for(int i = 0; i < 3; i++)
			{
				velocityProfileSpline(coeff_[i], actual_position[i], vel_[i], desired_position[i], 0.0, time);
			}
		} else {
			for(int i = 0; i < 3; i++)
			{
				velocityProfileLinear(coeff_[i], actual_position[i], desired_position[i], (double)macroSteps * MSTEP_TIME);
                vel_[i] = (desired_position[i] - actual_position[i]) / ((double)macroSteps * MSTEP_TIME);
			}
		}
		mstep_ = 1;
	}

	//for all of the axes...
	double time = (double)mstep_ * MSTEP_TIME;
	double t[6];
	generatePowers(5, time, t);
	for (int i = 0; i < 3; i++) {
		position[i] = t[0]*coeff_[i][0] +
                              t[1]*coeff_[i][1] +
                              t[2]*coeff_[i][2] +
                              t[3]*coeff_[i][3] +
                              t[4]*coeff_[i][4] +
                              t[5]*coeff_[i][5];
	}
	++mstep_; // increment macro step number

	// --------- send new position to the robot (EDP) ---------------
	position_matrix.set_from_xyz_angle_axis(lib::Xyz_Angle_Axis_vector(position));
	//send new position to the robot
	position_matrix.get_frame_tab(the_robot->ecp_command.arm.pf_def.arm_frame);
	// --------- send new position to the robot (EDP) (end) --------------

	if (neuron_sensor->positionRequested()  && !breaking_) {
		neuron_sensor->sendCurrentPosition(position[0], position[1], position[2]);
		neuron_sensor->get_reading();
	}

    if(breaking_ && (mstep_ > break_steps_)){
		return false;
	} else {
		return true;
	}
}

/**
 * @brief Returns current robot position.
 * @return Current robot position.
 */
double * neuron_generator::get_position()
{
	return position;
}

/**
 * @brief Returns time necessary to reach the desired position while breaking.
 * @return time necessary to reach the desired position while breaking (in seconds)
 */
double neuron_generator::get_breaking_time()
{
	return breaking_node * 0.02;
}

/**
 * @brief returns the overshoot.
 * @return the biggest value of the overshoot while breaking.
 */
double neuron_generator::get_overshoot()
{
	return overshoot;
}

/**
 * @brief Resets generator between consecutive call of Move() method.
 * @details Resets all of the temporary variables. It is necessary to call the
 * reset between the calls of the generator Move() method.
 */
void neuron_generator::reset()
{

	for (int i = 0; i < 6; i++) {
		v[i] = 0.0;
		u[i] = 0.0;
		a_max[i] = 0.15;
		v_max[i] = 0.15;
		s[i] = 0.0;
		k[i] = 0.0;
		reached[i] = false;
		change[i] = false;
		t = 0.02;
		almost_reached[i] = false;
		breaking_possible[i] = false;
		overshoot = 0;
	}

        breaking_ = false;
	breaking_node = 1;
}

void neuron_generator::velocityProfileLinear(double *coeff, double pos1, double pos2, double t)
{
	coeff[0] = pos1;
	coeff[1] = (pos2 - pos1) / t;
	coeff[2] = 0.0;
	coeff[3] = 0.0;
	coeff[4] = 0.0;
	coeff[5] = 0.0;
}

void neuron_generator::velocityProfileSpline(double *coeff, double pos1, double vel1, double pos2, double vel2, double time)
{
	double t[6];
	generatePowers(5, time, t);

	coeff[0] = pos1;
	coeff[1] = vel1;
	coeff[2] = (-3.0*pos1 + 3.0*pos2 - 2.0*vel1*t[1] - vel2*t[1]) / t[2];
	coeff[3] = (2.0*pos1 - 2.0*pos2 + vel1*t[1] + vel2*t[1]) / t[3];
	coeff[4] = 0.0;
	coeff[5] = 0.0;
}

}//generator
}//common
}//ecp
}//mrrocpp
