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
#include <math.h>

namespace mrrocpp {
    namespace ecp {
        namespace common {
            namespace generator {

                /*================================Constructor=============================*//**
 * @brief Constructor along with task configurator.
 * @param _ecp_task Reference to task configurator.
 */

                neuron_generator::neuron_generator(common::task::task& _ecp_task) :
                common::generator::generator(_ecp_task) {
                    reset();
                }

                /*==============================Destructor================================*//**
 * @brief Destructor.
 */

                neuron_generator::~neuron_generator() {
                }

                /*===============================first_step===============================*//**
 * @brief First step of neuron generator
 * @details Initializes instruction for robot and sends information to VSP that
 * new trajectory execution is starting, therefore requests first coordinates
 * of a trajectory.
 */

                bool neuron_generator::first_step() {
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
                    macroSteps = neuron_sensor->getMacroStepsNumber();
                    printf("macroStep: %d", macroSteps);

                    return true;
                }

                /*================================next_step===============================*//**
 * @brief next step for neuron generator.
 * @details Next step interpolates 5 consecutive macro steps basing on received
 * coordinates from VSP. It also receives information whether to start breaking
 * or not. More over it calculates the overshoot of the manipulator.
 */

                bool neuron_generator::next_step() {
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
                    if (neuron_sensor->current_period == macroSteps) {
                        //printf("period 5\n");
                        if (neuron_sensor->startBraking()) {
                            breaking = true;
                            printf("\n-------- breaking ----------\n");
                            first_breaking_node = true;
                            flushall();
                        }

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


                        //printf("current: %f\t %f\t %f\t %f\t %f\t %f\n", actual_position[0], actual_position[1], actual_position[2], actual_position[3], actual_position[4], actual_position[5]);
                        //printf("desired: %f\t %f\t %f\t %f\t %f\t %f\n", desired_position[0], desired_position[1], desired_position[2], desired_position[3], desired_position[4], desired_position[5]);

                        if (breaking) {
                            //			for (int i = 0; i < 3; i++) {
                            //				normalized_vector[i] = (last_but_one[i] - desired_position[i]) /
                            //						sqrt((last_but_one[0] - desired_position[0]) * (last_but_one[0] - desired_position[0]) +
                            //							 (last_but_one[1] - desired_position[1]) * (last_but_one[1] - desired_position[1]) +
                            //						     (last_but_one[2] - desired_position[2]) * (last_but_one[2] - desired_position[2]));
                            //			}
                            overshoot = (position[0] - desired_position[0]) * normalized_vector[0] + (position[1] - desired_position[1]) * normalized_vector[1] + (position[2] - desired_position[2]) * normalized_vector[2];
                            //printf("current: %f\t %f\t %f\t %f\t %f\t %f\n", actual_position[0], actual_position[1], actual_position[2], actual_position[3], actual_position[4], actual_position[5]);
                            //printf("desired: %f\t %f\t %f\t %f\t %f\t %f\n", desired_position[0], desired_position[1], desired_position[2], desired_position[3], desired_position[4], desired_position[5]);
                        }
                    }

                    int node = macroSteps + 1 - neuron_sensor->current_period;

                    if (breaking) {
                        //set the current period to 4 to avoid entering the above "if" condition.
                        neuron_sensor->current_period = macroSteps - 1;

                        if (first_breaking_node == true) {
                            printf("przed calculate\n");
                            flushall();
                            calculate();
                            first_breaking_node = false;
                            return false;
                        }
                    }

                    //for all of the axes...
                    for (int i = 0; i < 6; i++) {
                        //if no motion in the axis
                        if (desired_position[i] == actual_position[i] && !breaking) {

                            //position remains the same
                            position[i] = actual_position[i];

                            continue;
                        }

                        if (!breaking) {
                            if (desired_position[i] - actual_position[i] > 0) {
                                k[i] = 1;
                            } else {
                                k[i] = -1;
                            }
                        }

                        if (!breaking) {
                            u[i] = fabs(desired_position[i] - actual_position[i]);
                        }

                        if (breaking) {

                            //input position from the list

                        } else {
                            //normal motion (not breaking), distance between desired and
                            //current position is divided by 5, desired position is reached in
                            //5 macrosteps and added to actual position
                            position[i] = actual_position[i] + (k[i] * (u[i] / macroSteps) * node);
                            //current velocity, last v[i] is the velocity just before breaking,
                            //it is not updated during breaking
                            v[i] = (u[i] / macroSteps) / 0.02;
                            //printf("v[%d]: %f\n", i, v[i]);
                        }
                    }
                    //printf("\n");
                    /*printf("k:\t %d\t\t %d\t\t %d\t\t %d\t\t %d\t\t %d\n", k[0], k[1], k[2], k[3], k[4], k[5]);
                    printf("s:\t %f\t %f\t %f\t %f\t %f\t %f\n", s[0], s[1], s[2], s[3], s[4], s[5]);
                    printf("u:\t %f\t %f\t %f\t %f\t %f\t %f\n", u[0], u[1], u[2], u[3], u[4], u[5]);
                    printf("v:\t %f\t %f\t %f\t %f\t %f\t %f\n", v[0], v[1], v[2], v[3], v[4], v[5]);
                    printf("v_max:\t %f\t %f\t %f\t %f\t %f\t %f\n", v_max[0], v_max[1], v_max[2], v_max[3], v_max[4], v_max[5]);
                    printf("a_max:\t %f\t %f\t %f\t %f\t %f\t %f\n", a_max[0], a_max[1], a_max[2], a_max[3], a_max[4], a_max[5]);
                    printf("chan:\t %d\t\t %d\t\t %d\t\t %d\t\t %d\t\t %d\n", change[0], change[1], change[2], change[3], change[4], change[5]);
                    printf("rea:\t %d\t\t %d\t\t %d\t\t %d\t\t %d\t\t %d\n", reached[0], reached[1], reached[2], reached[3], reached[4], reached[5]);
                    printf("a_rea:\t %d\t\t %d\t\t %d\t\t %d\t\t %d\t\t %d\n", almost_reached[0], almost_reached[1], almost_reached[2], almost_reached[3], almost_reached[4], almost_reached[5]);
                    printf("b_p:\t %d\t\t %d\t\t %d\t\t %d\t\t %d\t\t %d\n", breaking_possible[0], breaking_possible[1], breaking_possible[2], breaking_possible[3], breaking_possible[4], breaking_possible[5]);*/


                    //increment the breaking node counter if in breaking phase
                    if (breaking) {
                        breaking_node++;
                    }

                    flushall();

                    if ((position[0] - desired_position[0]) * normalized_vector[0] + (position[1] - desired_position[1]) * normalized_vector[1] + (position[2] - desired_position[2]) * normalized_vector[2] < overshoot && breaking) {
                        overshoot = (position[0] - desired_position[0]) * normalized_vector[0] + (position[1] - desired_position[1]) * normalized_vector[1] + (position[2] - desired_position[2]) * normalized_vector[2];
                    }

                    //printf("overshoot: %f\n", overshoot);

                    //printf("pos:\t %f\t %f\t %f\t %f\t %f\t %f\n", position[0], position[1], position[2], position[3], position[4], position[5]);
                    // --------- send new position to the robot (EDP) ---------------
                    position_matrix.set_from_xyz_angle_axis(lib::Xyz_Angle_Axis_vector(position));
                    //send new position to the robot
                    position_matrix.get_frame_tab(the_robot->ecp_command.arm.pf_def.arm_frame);
                    // --------- send new position to the robot (EDP) (end) --------------

                    if (neuron_sensor->current_period == 1) {
                        //printf("coordiantes sent from current period = 1\n");
                        neuron_sensor->sendCurrentPosition(position[0], position[1], position[2]);
                    }

                    if (breaking) {
                        //check if it is the last macrostep in the list
                        return false;
                    }

                    //return false, desired positions in all of the axes reached
                    return true;
                }

                void neuron_generator::clear_vectors() {
                    s_vect.clear();
                    v_max_vect.clear();
                    a_max_vect.clear();
                    desired_position_vect.clear();
                    actual_position_vect.clear();
                }

                void neuron_generator::calculate() {
                    printf("calculate\n");
                    flushall();
                    int i;
                    for (i = 0; i < 3; i++) {

                        clear_vectors();
                        printf("after clear vect\n");
                        flushall();
                        s[i] = fabs(actual_position[i] - desired_position[i]);

                        actual_position_vect.push_back(actual_position[i]);

                        std::vector <ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose> new_pose_vector = std::vector <ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose>();
                        pose_vector_list.push_back(new_pose_vector);

                        printf("before if\n");
                        flushall();
                        
                        if (check_if_able_to_break(s[i], v[i], a_max[i])) {//one pose breaking

                            printf("jedna pozycja\n");
                            flushall();
                            s_vect.push_back(s[i]);
                            v_max_vect.push_back(v_max[i]);
                            a_max_vect.push_back(a_max[i]);
                            desired_position_vect.push_back(desired_position[i]);
                            load_trajectory_pose(pose_vector_list[i], desired_position_vect, lib::ABSOLUTE, lib::ECP_XYZ_ANGLE_AXIS,
                                    v_max_vect, a_max_vect, actual_position_vect, s_vect);

                            pose_vector_iterator = pose_vector_list[i].begin();

                            pose_vector_iterator->k[0] = k[i];
                            pose_vector_iterator->v_p[0] = v[i];
                            pose_vector_iterator->v_k[0] = 0.0;

                            vpc.set_model_pose(pose_vector_iterator);
                            vpc.calculate_s_acc_s_dec_pose(pose_vector_iterator);

                            if (vpc.check_s_acc_s_decc(pose_vector_iterator, 0)) {//check if s_acc && s_dec < s
                                vpc.calculate_s_uni(pose_vector_iterator, 0); //calculate s_uni
                                vpc.calculate_time(pose_vector_iterator, 0); //calculate and set time
                            } else {//if not
                                if (!vpc.optimize_time_axis(pose_vector_iterator, 0)) {
                                    printf("optimize time error\n");
                                    flushall();
                                }

                                if (!vpc.reduction_axis(pose_vector_iterator, 0)) {
                                    printf("reduction axis \n");
                                    flushall();
                                }
                            }

                            pose_vector_iterator->t = pose_vector_iterator->times[0];
                            time_sum.push_back(pose_vector_iterator->times[0]);

                        } else {//2 pose breaking
                            printf("dwie pozycje\n");
                            flushall();
                            //first pose
                            s[i] = 0.5 * v[i] * v[i] / a_max[i];
                            double t_temp = sqrt(2 * s[i] / a_max[i]);

                            if (ceil(t_temp / t) * t != t_temp) { //extend the pose time to be the multiplicity of the macrostep time
                                t_temp = ceil(t_temp / t);
                                t_temp = t_temp * t;
                            }

                            double temp_a_max = 2 * s[i] / (t_temp * t_temp);

                            v_max_vect.push_back(v_max[i]);
                            a_max_vect.push_back(temp_a_max);

                            s_vect.push_back(s[i]);
                            desired_position_vect.push_back(actual_position[i] + k[i] * s[i]);

                            printf("before load first pose\n");
                            flushall();

                            load_trajectory_pose(pose_vector_list[i], desired_position_vect, lib::ABSOLUTE, lib::ECP_XYZ_ANGLE_AXIS,
                                    v_max_vect, a_max_vect, actual_position_vect, s_vect);

                            printf("before iterator settinf\n");
                            flushall();
                            pose_vector_iterator = pose_vector_list[i].begin();

                            printf("before other settings\n");
                            flushall();
                            printf("pose v_r: %f\n", pose_vector_iterator->v_r[0]);
                            flushall();

                            pose_vector_iterator->k[0] = k[i];
                            printf("settings 1\n");
                            flushall();
                            pose_vector_iterator->v_p[0] = v[i];
                            printf("settings 2\n");
                            flushall();
                            pose_vector_iterator->v_k[0] = 0.0;
                            printf("settings 3\n");
                            flushall();
                            pose_vector_iterator->times[0] = t_temp;
                            printf("settings 4\n");
                            flushall();
                            pose_vector_iterator->s_acc[0] = s[i];
                            printf("settings 5\n");
                            flushall();
                            pose_vector_iterator->s_dec[0] = 0.0;
                            printf("settings 6\n");
                            flushall();
                            pose_vector_iterator->s_uni[0] = 0.0;
                            printf("settings 7\n");
                            flushall();
                            pose_vector_iterator->t = t_temp;
                            printf("settings 8\n");
                            flushall();
                            pose_vector_iterator->v_r[0] = 0.0;
                            printf("settings 9\n");
                            flushall();
                            pose_vector_iterator->interpolation_node_no = ceil(pose_vector_iterator->t / t);
                            printf("settings 10\n");
                            flushall();
                            pose_vector_iterator->acc[0] = pose_vector_iterator->interpolation_node_no;
                            printf("settings 11\n");
                            flushall();
                            pose_vector_iterator->uni[0] = 0.0;

                            printf("before time_sum push back\n");
                            flushall();
                            time_sum.push_back(pose_vector_iterator->times[0]);
                            //first pose end

                            printf("po pierwszej poz\n");
                            flushall();
                            
                            clear_vectors();

                            v_max_vect.push_back(v_max[i]);
                            a_max_vect.push_back(a_max[i]);
                            s[i] = fabs(desired_position_vect[0] - desired_position[i]);
                            actual_position_vect.push_back(desired_position_vect[0]);
                            desired_position_vect.push_back(desired_position[i]);
                            s_vect.push_back(s[i]);
                            //second pose

                            printf("przed drugą poz\n");
                            flushall();
                            load_trajectory_pose(pose_vector_list[i], desired_position_vect, lib::ABSOLUTE, lib::ECP_XYZ_ANGLE_AXIS,
                                    v_max_vect, a_max_vect, actual_position_vect, s_vect);


                            printf("pose_vector size 2: %d\n", pose_vector_list[i].size());
                            pose_vector_iterator++;

                            printf("przed drugimi settings\n");
                            printf("v_r[0]: %f\n", pose_vector_iterator->v_r[0]);
                            flushall();

                            pose_vector_iterator->k[0] = -k[i];
                            printf("k[0]: %f\n", pose_vector_iterator->k[0]);
                            flushall();
                            pose_vector_iterator->v_p[0] = 0.0;
                            pose_vector_iterator->v_k[0] = 0.0;
                            pose_vector_iterator->model[0] = 1;

                            printf("przed calculate s acc s dec\n");
                            flushall();
                            vpc.calculate_s_acc_s_dec_pose(pose_vector_iterator);

                            if (vpc.check_s_acc_s_decc(pose_vector_iterator, 0)) {//check if s_acc && s_dec < s
                                vpc.calculate_s_uni(pose_vector_iterator, 0); //calculate s_uni
                                vpc.calculate_time(pose_vector_iterator, 0); //calculate and set time
                            } else {//if not
                                if (!vpc.optimize_time_axis(pose_vector_iterator, 0)) {
                                    printf("optimize time error\n");
                                    flushall();
                                }

                                if (!vpc.reduction_axis(pose_vector_iterator, 0)) {
                                    printf("reduction axis error\n");
                                    flushall();
                                }
                            }
                            time_sum[i] += pose_vector_iterator->times[0];
                            //second pose end
                        }
                    }

                    double max_time = 0.0;
                    
                    for (i = 0; i < 3; i++) {
                        
                        if (max_time < time_sum[i]) {
                            max_time = time_sum[i];
                        }
                    }

                    if (ceil(max_time / t) * t != max_time) { //extend the pose time to be the multiplicity of the macrostep time
                        max_time = ceil(max_time / t);
                        max_time = max_time * t;
                    }

                    for (i = 0; i < 3; i ++) {
                        pose_vector_iterator = pose_vector_list[i].begin();

                        if (pose_vector_iterator++ == pose_vector_list[i].end()) {
                            pose_vector_iterator--;
                            printf("jedna pozycja 2\n");
                            flushall();
                            pose_vector_iterator->t = max_time;
                            vpc.set_times_to_t(pose_vector_iterator);
                            pose_vector_iterator->interpolation_node_no = ceil(pose_vector_iterator->t / t);
                        } else {
                            pose_vector_iterator--;
                            printf("dwie pozycje 2\n");
                            flushall();
                            double temp_time = pose_vector_iterator->t;
                            pose_vector_iterator++;
                            pose_vector_iterator->t = max_time - temp_time;
                            vpc.set_times_to_t(pose_vector_iterator);
                            pose_vector_iterator->interpolation_node_no = ceil(pose_vector_iterator->t / t);
                        }

                        if(!vpc.reduction_axis(pose_vector_iterator, 0)) {
                            printf("reduction axis 2 error\n");
                            flushall();
                        }
                        if (!vpc.calculate_acc_uni_pose(pose_vector_iterator, t)) {//set uni and acc
                            printf("calculate acc_uni_pose error\n");
                            flushall();
                        }
                    }
                }

                bool neuron_generator::check_if_able_to_break(double s, double v, double a_max) {
                    if ((0.5 * v * v / a_max) > s) {
                        return false;
                    } else {
                        return true;
                    }
                }

                bool neuron_generator::load_trajectory_pose(std::vector<ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose> & pose_vector,
                        const std::vector<double> & coordinates, lib::MOTION_TYPE motion_type, lib::ECP_POSE_SPECIFICATION pose_spec,
                        const std::vector<double> & v, const std::vector<double> & a, const std::vector<double> & start_pos,
                        const std::vector<double> & s) {

                    printf("load pose\n");
                    flushall();
                    ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose pose; //new trajectory pose
                    pose = ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose(pose_spec, coordinates, v, a); //create new trajectory pose
                    pose.v_r = v; //set the v_max vector
                    pose.a_r = a; //set the a_max vector

                    printf("load pose 2\n");
                    flushall();
                    if (pose_vector.empty()) {
                        pose.pos_num = 1;
                    } else {
                        pose.pos_num = pose_vector.back().pos_num + 1;
                    }

                    printf("load pose 3\n");
                    flushall();
                    if (motion_type == lib::ABSOLUTE) {
                        if (!pose_vector.empty()) {//set the start position of the added pose as the desired position of the previous pose
                            pose.start_position = pose_vector.back().coordinates;
                        } else {
                            pose.start_position = start_pos;
                        }
                    }

                    printf("load pose 4\n");
                    flushall();
                    pose.s = s;

                    printf("pose przed pushem: %f\n", pose.v_r[0]);
                    flushall();
                    pose_vector.push_back(pose); //put new trajectory pose into a pose vector

                    printf("v_r w load pose: %f\n", pose_vector[0].v_r[0]);
                    flushall();

                    return true;
                }

                /**
                 * @brief Returns current robot position.
                 * @return Current robot position.
                 */
                double * neuron_generator::get_position() {
                    return position;
                }

                /**
                 * @brief Returns time necessary to reach the desired position while breaking.
                 * @return time necessary to reach the desired position while breaking (in seconds)
                 */
                double neuron_generator::get_breaking_time() {
                    return breaking_node * 0.02;
                }

                /**
                 * @brief returns the overshoot.
                 * @return the biggest value of the overshoot while breaking.
                 */
                double neuron_generator::get_overshoot() {
                    return overshoot;
                }

                /**
                 * @brief Resets generator between consecutive call of Move() method.
                 * @details Resets all of the temporary variables. It is necessary to call the
                 * reset between the calls of the generator Move() method.
                 */
                void neuron_generator::reset() {

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
                        first_breaking_node = false;
                    }

                    breaking = false;
                    breaking_node = 1;
                }

            }//generator
        }//common
    }//ecp
}//mrrocpp
