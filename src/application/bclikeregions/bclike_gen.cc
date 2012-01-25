/**
 * \file bclike_gen.cc
 * \brief Scanning subtask generator class methods definition file
 * \date 02.09.2010
 * \author Kacper Szkudlarek
 */

#include "bclike_gen.h"
#include "bclikeregions_task.h"
#include "bcl_t_switcher.h"
#include <cstring>

namespace mrrocpp {

namespace ecp {

namespace common {

namespace generator {

//Rotation 180 x, 90 z
const double rotation[3][3] = {{-1,    0,   0},
							   { 0,   -1,   0},
							   { 0,    0,   1}};

//Camera translation matrix
const double translation[3] = {0, 0, 0};


#ifdef IRP6_OT
const int joint_num = 6;
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



bclike_gen::bclike_gen(mrrocpp::ecp::common::task::task & ecp_task) :
		mrrocpp::ecp::common::generator::constant_velocity(ecp_task, move_type, joint_num),
		bcl_ecp((task::bcl_t_switcher &)ecp_t), num_send(0){

	vsp_fradia = NULL;
	no_fradia = true;

}


bclike_gen::bclike_gen(mrrocpp::ecp::common::task::bcl_t_switcher & task):
				common::generator::constant_velocity((mrrocpp::ecp::common::task::task &)task, move_type, joint_num),
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


bclike_gen::bclike_gen(mrrocpp::ecp::common::task::bcl_t_switcher & task, task::bcl_fradia_sensor* fr):
						common::generator::constant_velocity((mrrocpp::ecp::common::task::task &)task, move_type, joint_num),
						bcl_ecp((task::bcl_t_switcher &)ecp_t),
						vsp_fradia(fr), num_send(0){

	no_fradia = false;

	if(vsp_fradia != NULL){
		sensor_m[ecp_mp::sensor::SENSOR_FRADIA] = vsp_fradia;
		vsp_fradia->base_period = 1;

		std::cout << "SENSOR ADD vsp constructor" << std::endl;
	}
}

bclike_gen::~bclike_gen() {
}


bool bclike_gen::first_step(){

	the_robot->ecp_command.instruction_type = lib::GET;
	the_robot->ecp_command.get_type = ARM_DEFINITION;
	the_robot->ecp_command.get_arm_type = return_pos_type;

	if(no_fradia){
		std::cout << "ERROR: no fradia == TRUE" << std::endl;
		return false;
	}

//	return newsmooth::first_step();
	return constant_velocity::first_step();
}

bool bclike_gen::next_step(){

	//Get FraDIA reading
	reading = bcl_ecp.vsp_fradia->get_reading_message();

	//Get actual robot's position
	actual_pos = the_robot->reply_package.arm.pf_def.arm_frame;
	std::vector<double> vec;
	vec.assign(the_robot->reply_package.arm.pf_def.arm_coordinates, the_robot->reply_package.arm.pf_def.arm_coordinates + VEC_SIZE);

	//If there are new bar code like areas translate their positions and check existance in vector
	if(reading.code_found){
		reading.code_found = false;
		double t[3];
		actual_pos.get_translation_vector(t);
		translateToRobotPosition(reading);
		addCodesToVector(reading);
//		std::cout << "KODOW DO WYSANIA: " << readings.size() << std::endl;
	}

	//If there is something to send, do it
	if(sendNextPart()){
//		newsmooth::reset();
		return false;
	}

	//If there is nothing to send and robot is still moving, go on
//	if(newsmooth::next_step()){
//		return true;
//	}

	if(constant_velocity::next_step()){
		return true;
	}

	//End everything, when there is nothing to send and robot stops
	strcpy(ecp_t.ecp_reply.recognized_command, "KONIEC");
	readings.clear();
	return false;


}


void bclike_gen::translateToRobotPosition(task::fradia_regions& regs){

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
			case 0:
				e(0, 0) = regs.x_k0;
				e(1, 0) = regs.y_k0;
				break;
			case 1:
				e(0, 0) = regs.x_k1;
				e(1, 0) = regs.y_k1;
				break;
			case 2:
				e(0, 0) = regs.x_k2;
				e(1, 0) = regs.y_k2;
				break;
			case 3:
				e(0, 0) = regs.x_k3;
				e(1, 0) = regs.y_k3;
				break;

		}
		e(2, 0) = 0;
		e(3, 0) = 0;


		e_translation(0, 0) = e(0, 0);
		e_translation(1, 0) = e(1, 0);
		e_translation(2, 0) = e(2, 0);

	//		[0.00004 0 0 0 0 0; 0 0.00004 0 0 0 0; 0 0 0.001 0 0 0; 0 0 0 0.1 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0]

		Kp(0,0) = 0.000476;
		Kp(1,1) = 0.000476;
		Kp(2,2) = 0.001;
		Kp(3,3) = 0.1;

		control = Kp * e;

		camera_to_object_translation(0, 0) = control(0, 0);
		camera_to_object_translation(1, 0) = control(1, 0);
		camera_to_object_translation(2, 0) = control(2, 0);

		e_T_c_position.remove_translation();
		e_T_c_position.remove_rotation();

		e_T_c_position.set_rotation_matrix(rotation);
		e_T_c_position.set_translation_vector(translation);

		u_translation = e_T_c_position * camera_to_object_translation;

		u_rotation.set_from_xyz_angle_axis(lib::Xyz_Angle_Axis_vector(0, 0, 0, 0, 0, control(3, 0)));

		delta_position.set_rotation_matrix(u_rotation);
		delta_position.set_translation_vector(u_translation);

		tmp_pos = actual_pos * delta_position;

		tmp_pos.get_xyz_euler_zyz(new_pos);

		switch(i){
			case 0:
				regs.x_k0 = new_pos(0,0);
				regs.y_k0 = new_pos(1,0);
				regs.r_k0 = regs.r_k0 * 0.1 / 210;
				break;
			case 1:
				regs.x_k1 = new_pos(0,0);
				regs.y_k1 = new_pos(1,0);
				regs.r_k1 = regs.r_k1 * 0.1 / 210;
				break;
			case 2:
				regs.x_k2 = new_pos(0,0);
				regs.y_k2 = new_pos(1,0);
				regs.r_k2 = regs.r_k2 * 0.1 / 210;
				break;
			case 3:
				regs.x_k3 = new_pos(0,0);
				regs.y_k3 = new_pos(1,0);
				regs.r_k3 = regs.r_k3 * 0.1 / 210;
				break;
		}
	}

}


void bclike_gen::addCodesToVector(task::fradia_regions reading){

	task::mrrocpp_regions tmp;

	switch(reading.num_found){
		case 4:
			tmp.x = reading.x_k3;
			tmp.y = reading.y_k3;
			tmp.r = reading.r_k3;
			if(!checkIfCodeBeenRead(tmp))
				readings.push_back(std::pair<task::mrrocpp_regions, bool>(tmp, false));
		case 3:
			tmp.x = reading.x_k2;
			tmp.y = reading.y_k2;
			tmp.r = reading.r_k2;
			if(!checkIfCodeBeenRead(tmp))
				readings.push_back(std::pair<task::mrrocpp_regions, bool>(tmp, false));
		case 2:
			tmp.x = reading.x_k1;
			tmp.y = reading.y_k1;
			tmp.r = reading.r_k1;
			if(!checkIfCodeBeenRead(tmp))
				readings.push_back(std::pair<task::mrrocpp_regions, bool>(tmp, false));
		case 1:
			tmp.x = reading.x_k0;
			tmp.y = reading.y_k0;
			tmp.r = reading.r_k0;
			if(!checkIfCodeBeenRead(tmp))
				readings.push_back(std::pair<task::mrrocpp_regions, bool>(tmp, false));
			break;
		case 0:
			break;
	}

}


bool bclike_gen::checkIfCodeBeenRead(task::mrrocpp_regions& code){

	std::vector<std::pair<task::mrrocpp_regions, bool> >::iterator it;

	for(it = readings.begin(); it != readings.end(); ++ it){
		if(codesIntersect(code, (*it).first)){
			(*it).first.x = ((*it).first.x + code.x)/2;
			(*it).first.y = ((*it).first.y + code.y)/2;
			//(*it).first.r = sqrt(((*it).first.x - code.x)*((*it).first.x - code.x) + ((*it).first.y - code.y)*((*it).first.y - code.y))/2 + ((*it).first.r + code.r)/2;
			(*it).first.r = hypot(((*it).first.x - code.x), ((*it).first.y - code.y))/2 + ((*it).first.r + code.r)/2;

			return true;
		}
	}

	return false;
}

bool bclike_gen::codesIntersect(task::mrrocpp_regions& c1, task::mrrocpp_regions& c2){

	//if(sqrt((c1.x - c2.x)*(c1.x - c2.x) + (c1.y - c2.y)*(c1.y - c2.y)) < (c1.r + c2.r)){
	if(hypot((c1.x - c2.x), (c1.y - c2.y)) < (c1.r + c2.r)){
		return true;
	}
	return false;
}



bool bclike_gen::sendNextPart(){

	char* ret = new char[lib::MP_2_ECP_SERIALIZED_DATA_SIZE];
	double *tab = reinterpret_cast<double*>(ret);

	//Write to matrix number of elements which will be written to
	lib::Xyz_Euler_Zyz_vector new_pos;
	std::vector<double> vec;
	actual_pos.get_xyz_euler_zyz(new_pos);
	new_pos.to_vector(vec);
	tab[0] = vec.size();

	int i = 1;
	//Rewrite vector elements to matrix
	for(std::vector<double>::iterator it = vec.begin(); (it != vec.end()) && (i < lib::MP_2_ECP_SERIALIZED_DATA_SIZE/sizeof(double)); ++it, ++i){
		tab[i] = *it;
	}

	std::vector<std::pair<task::mrrocpp_regions, bool> >::iterator it;// = readings.begin();

	tab[i] = 0;

	for(it = readings.begin(); it != readings.end() && ((i + 5 * tab[i] + 1) * sizeof(double) < lib::MP_2_ECP_SERIALIZED_DATA_SIZE); ++it){
		if(!(*it).second){
			tab[i + 3 * (int)tab[i] + 1] = (*it).first.x;
			tab[i + 3 * (int)tab[i] + 2] = (*it).first.y;
			tab[i + 3 * (int)tab[i] + 3] = (*it).first.r;
			tab[i]++;
			(*it).second = true;
		}
	}


	if(tab[i] > 0){
		memcpy(ecp_t.ecp_reply.recognized_command, ret, sizeof(char) * lib::MP_2_ECP_SERIALIZED_DATA_SIZE);
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
