/*
 * bclike_smooth.cpp
 *
 *  Created on: 05-07-2010
 *      Author: kszkudla
 */

#include "bclike_smooth.h"
#include "bclikeregions_task.h"
#include "bcl_t_switcher.h"
#include <cstring>

namespace mrrocpp {

namespace ecp {

namespace common {

namespace generator {


#ifdef IRP6_OT
const int joint_num = 7;
#endif//IRP6_OT

#ifdef IRP6_P
const int joint_num = 6;
#endif //IRP6_P

#ifdef JOINT
const  lib::ECP_POSE_SPECIFICATION move_type = lib::ECP_JOINT;
const lib::POSE_SPECIFICATION return_pos_type = lib::FRAME;
#endif //JOINT

#ifdef EULER
const lib::ECP_POSE_SPECIFICATION move_type = lib::ECP_XYZ_EULER_ZYZ;
const lib::POSE_SPECIFICATION return_pos_type = lib::FRAME;
#endif //EULER


/**
 * Class constructor without creating FraDIA sensor
 * @param ecp_task parent task
 */
bclike_smooth::bclike_smooth(mrrocpp::ecp::common::task::task & ecp_task) :
		common::generator::newsmooth(ecp_task, move_type, joint_num),
		bcl_ecp((task::bcl_t_switcher &)ecp_t), num_send(0){

	vsp_fradia = NULL;
	no_fradia = true;

}

/**
 * Class constructor, FraDIA sensor is acquired from parent task
 * @param task parent task
 */
bclike_smooth::bclike_smooth(mrrocpp::ecp::common::task::bcl_t_switcher & task):
				common::generator::newsmooth((mrrocpp::ecp::common::task::task &)task, move_type, joint_num),
				bcl_ecp((task::bcl_t_switcher &)ecp_t), num_send(0){

	std::cout << "FRADIA VERSION" << std::endl;
	no_fradia = false;
	vsp_fradia = bcl_ecp.get_vsp_fradia();


	if(vsp_fradia != NULL){
		sensor_m[ecp_mp::sensor::SENSOR_FRADIA] = vsp_fradia;
		vsp_fradia->base_period = 1;

		std::cout << "SENSOR ADD no vsp constructor" << std::endl;
	}

}


/**
 * Class constructor, FraDIA sensor is given as a parameter
 * @param task parent task
 * @param fr pointer to FraDIA sensor structure
 */
bclike_smooth::bclike_smooth(mrrocpp::ecp::common::task::bcl_t_switcher & task, task::bcl_fradia_sensor* fr):
						common::generator::newsmooth((mrrocpp::ecp::common::task::task &)task, move_type, joint_num),
						bcl_ecp((task::bcl_t_switcher &)ecp_t),
						vsp_fradia(fr), num_send(0){

	no_fradia = false;

	if(vsp_fradia != NULL){
		sensor_m[ecp_mp::sensor::SENSOR_FRADIA] = vsp_fradia;
		vsp_fradia->base_period = 1;

		std::cout << "SENSOR ADD vsp constructor" << std::endl;
	}
}

bclike_smooth::~bclike_smooth() {
}

/**
 * Set necessary instructions, and other data for preparing the robot to move
 */
bool bclike_smooth::first_step(){

	std::cout << "FIRST STEP" << std::endl;

	the_robot->ecp_command.instruction.instruction_type = lib::GET;
	the_robot->ecp_command.instruction.get_type = ARM_DEFINITION;
	the_robot->ecp_command.instruction.get_arm_type = return_pos_type;

	if(no_fradia){
		std::cout << "ERROR: no fradia == TRUE" << std::endl;
		return false;
	}

	return newsmooth::first_step();
}

bool bclike_smooth::next_step(){

	//Get FraDIA reading
	reading = bcl_ecp.vsp_fradia->get_reading_message();

	//Get actual robot's position
	actual_pos.set_from_frame_tab(the_robot->reply_package.arm.pf_def.arm_frame);
	std::vector<double> vec;
	vec.assign(the_robot->reply_package.arm.pf_def.arm_coordinates, the_robot->reply_package.arm.pf_def.arm_coordinates + VEC_SIZE);

	//If there are new bar code like areas translate their positions and check existance in vector
	if(reading.code_found){
		double t[3];
		translateToRobotPosition(reading);
		std::cout << " x = " <<  reading.x_k0 << " y = " << reading.y_k0 << std::endl;
		actual_pos.get_translation_vector(t);
		std::cout << "ROBOT x = " <<  t[0] << "ROBOT y = " << t[1] << std::endl;
		addCodesToVector(reading);
	}

	//If there is something to send, do it
	if(sendNextPart())
		return false;

	//If there is nothing to send and robot is still moving, go on
	if(newsmooth::next_step())
		return true;

	//End everything, when there is nothing to send and robot stops
	strcpy(ecp_t.ecp_reply.ecp_2_mp_string, "KONIEC");
	return false;


}

/**
 * Translating code positions from local image position, to global robot positon
 * @param regs packet received from FraDIA
 */
void bclike_smooth::translateToRobotPosition(task::fradia_regions& regs){

	lib::K_vector u_translation(0, 0, 0);
	lib::Homog_matrix u_rotation;
	Eigen::Matrix <double, 6, 1> e;
	Eigen::Matrix <double, 3, 1> e_translation;

	Eigen::Matrix <double, 6, 1> control;
	Eigen::Matrix <double, 6, 6> Kp;

	Eigen::Matrix <double, 3, 1> camera_to_object_translation;

	lib::Homog_matrix e_T_c_position;

	lib::Homog_matrix delta_position;

	lib::Xyz_Euler_Zyz_vector new_pos;

	for(int i = 0; i < regs.num_found; ++i){

		e.setZero();
		Kp.setZero();
		e_translation.setZero();
		control.setZero();
		camera_to_object_translation.setZero();

		switch(i){
			case 1:
				e(0, 0) = regs.x_k0;
				e(1, 0) = regs.y_k0;
				break;
			case 2:
				e(0, 0) = regs.x_k1;
				e(1, 0) = regs.y_k1;
				break;
			case 3:
				e(0, 0) = regs.x_k2;
				e(1, 0) = regs.y_k2;
				break;

		}
		e(2, 0) = 0;
		e(3, 0) = 0;


		e_translation(0, 0) = e(0, 0);
		e_translation(1, 0) = e(1, 0);
		e_translation(2, 0) = e(2, 0);

	//		[0.00004 0 0 0 0 0; 0 0.00004 0 0 0 0; 0 0 0.001 0 0 0; 0 0 0 0.1 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0]

		Kp(0,0) = 0.00004;
		Kp(1,1) = 0.00004;
		Kp(2,2) = 0.001;
		Kp(3,3) = 0.1;

		control = Kp * e;

		camera_to_object_translation(0, 0) = control(0, 0);
		camera_to_object_translation(1, 0) = control(1, 0);
		camera_to_object_translation(2, 0) = control(2, 0);

		e_T_c_position.remove_translation();
		e_T_c_position.remove_rotation();
		Kp(0,0) = 1;
		Kp(1,1) = 1;
		Kp(2,2) = 1;

		u_translation = e_T_c_position * camera_to_object_translation;

		u_rotation.set_from_xyz_angle_axis(lib::Xyz_Angle_Axis_vector(0, 0, 0, 0, 0, control(3, 0)));

		delta_position.set_rotation_matrix(u_rotation);
		delta_position.set_translation_vector(u_translation);

		tmp_pos = actual_pos * delta_position;

		tmp_pos.get_xyz_euler_zyz(new_pos);


		switch(i){
			case 1:
				regs.x_k0 = new_pos(1,1);
				regs.y_k0 = new_pos(2,1);
				break;
			case 2:
				regs.x_k1 = new_pos(1,1);
				regs.y_k1 = new_pos(2,1);
				break;
			case 3:
				regs.x_k2 = new_pos(1,1);
				regs.y_k2 = new_pos(2,1);
				break;
		}
	}

}

/**
 * Function rewriting codes received from FraDIA to local container if they haven't been
 * there earlier
 * @param reading packet received from FraDIA framework
 */
void bclike_smooth::addCodesToVector(task::fradia_regions reading){

	task::mrrocpp_regions tmp;

	switch(reading.num_found){
		case 3:
			tmp.x = reading.x_k2;
			tmp.y = reading.y_k2;
			tmp.h = reading.h_k2;
			tmp.w = reading.w_k2;
			tmp.a = reading.a_k2;
			if(!checkIfCodeBeenRead(tmp))
				readings.push_back(std::pair<task::mrrocpp_regions, bool>(tmp, false));
		case 2:
			tmp.x = reading.x_k1;
			tmp.y = reading.y_k1;
			tmp.h = reading.h_k1;
			tmp.w = reading.w_k1;
			tmp.a = reading.a_k1;
			if(!checkIfCodeBeenRead(tmp))
				readings.push_back(std::pair<task::mrrocpp_regions, bool>(tmp, false));
		case 1:
			tmp.x = reading.x_k0;
			tmp.y = reading.y_k0;
			tmp.h = reading.h_k0;
			tmp.w = reading.w_k0;
			tmp.a = reading.a_k0;
			if(!checkIfCodeBeenRead(tmp))
				readings.push_back(std::pair<task::mrrocpp_regions, bool>(tmp, false));
			break;
		case 0:
			break;
	}
}

/**
 * Function to check if found code isn't already in memory vector
 * @param code Code which will be check if it intersect with any other code in vector
 * @return true if code is in vector, false otherwise
 */
bool bclike_smooth::checkIfCodeBeenRead(task::mrrocpp_regions& code){

	std::vector<std::pair<task::mrrocpp_regions, bool> >::iterator it;

	for(it = readings.begin(); it != readings.end(); ++ it){
		if(codesIntersect(code, (*it).first))
			return false;
	}

	return true;
}
/**
 * Check if two given code areas intersects
 * @param c1 first of codes to be checked
 * @param c2 second of codes to be checked
 * @return true if codes intersect, false otherwise
 */
bool bclike_smooth::codesIntersect(task::mrrocpp_regions& c1, task::mrrocpp_regions& c2){

	if(((c1.x > c2.x) && (c1.x + c1.w < c2.x + c2.w) && (c1.y > c2.y) && (c1.y + c1.h < c2.y + c2.h)) ||
	   ((c2.x > c1.x) && (c2.x + c2.w < c1.x + c1.w) && (c2.y > c1.y) && (c2.y + c2.h < c1.y + c1.h))){
		return true;
	}
	return false;
}


/**
 * Rewriting data from vector to buffer to send to MP
 */
bool bclike_smooth::sendNextPart(){

	char* ret = new char[MP_2_ECP_STRING_SIZE];
	double *tab = reinterpret_cast<double*>(ret);

	//Write to matrix number of elements which will be written to
	lib::Xyz_Euler_Zyz_vector new_pos;
	std::vector<double> vec;
	actual_pos.get_xyz_euler_zyz(new_pos);
	new_pos.to_vector(vec);
	tab[0] = vec.size();

	int i = 1;
	//Rewrite vector elements to matrix
	for(std::vector<double>::iterator it = vec.begin(); (it != vec.end()) && (i < MP_2_ECP_STRING_SIZE/sizeof(double)); ++it, ++i){
		tab[i] = *it;
	}

	std::vector<std::pair<task::mrrocpp_regions, bool> >::iterator it;// = readings.begin();

	tab[i] = 0;

	for(it = readings.begin(); it != readings.end() && ((5 * tab[i] + 1) * sizeof(double) < MP_2_ECP_STRING_SIZE); ++it){
		if(!(*it).second){
			tab[i + 5 * (int)tab[i] + 1] = (*it).first.x;
			tab[i + 5 * (int)tab[i] + 2] = (*it).first.y;
			tab[i + 5 * (int)tab[i] + 3] = (*it).first.w;
			tab[i + 5 * (int)tab[i] + 4] = (*it).first.h;
			tab[i + 5 * (int)tab[i] + 5] = (*it).first.a;
			tab[i]++;
			(*it).second = true;
		}
	}

	if((int)tab[i]){
		strcpy(ecp_t.ecp_reply.ecp_2_mp_string, ret);
		delete(ret);
		return true;
	}else{
		delete(ret);
		return false;
	}
}

}

}

}

}
