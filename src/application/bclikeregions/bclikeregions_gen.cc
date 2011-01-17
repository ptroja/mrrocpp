///*
// * bclikeregions_gen.cc
// *
// *  Created on: May 18, 2010
// *      Author: kszkudla
// */
//
//#include "bclikeregions_gen.h"
//
//namespace mrrocpp {
//
//namespace ecp {
//
//namespace common {
//
//namespace generator {
//
//#define POS(tab, x, y) tab[x + y * 3]
//
////#define M_PI 3.14159
//
//#define RTOD(x) (x*180/M_PI)
//
//#define DTOR(x) (x*M_PI/180)
//
//bclikeregions_gen::bclikeregions_gen(mrrocpp::ecp::common::task::task & ecp_task):
//		generator(ecp_task), r(0.15) {
//
//	// TODO Auto-generated constructor stub
//	motion_steps = 30;
//
//	angle = 0.01;
//
//	first_pos_saved = false;
//
//	Eigen::Matrix <double, 3, 1> p1, p2;
//		p1(0, 0) = 0.6;
//		p1(1, 0) = -0.3;
//		p1(2, 0) = 0.1;
//
//		p2(0, 0) = 0.98;
//		p2(1, 0) = 0.3;
//		p2(2, 0) = 0.3;
//
//
//}
//
//bclikeregions_gen::~bclikeregions_gen() {
//
//	// TODO Auto-generated destructor stub
//
//}
//
//bool bclikeregions_gen::first_step(){
//
//	the_robot->ecp_command.instruction.instruction_type = lib::GET;
//	the_robot->ecp_command.instruction.get_type = ARM_DEFINITION;
//	the_robot->ecp_command.instruction.get_arm_type = lib::FRAME;
//	the_robot->ecp_command.instruction.motion_type = lib::ABSOLUTE;
//	the_robot->ecp_command.instruction.set_type = ARM_DEFINITION;
//	the_robot->ecp_command.instruction.set_arm_type = lib::FRAME;
//	the_robot->ecp_command.instruction.interpolation_type = lib::TCIM;
//	the_robot->ecp_command.instruction.motion_steps = motion_steps;
//	the_robot->ecp_command.instruction.value_in_step_no = motion_steps - 3;
//
//	if(!first_pos_saved){
//		first_pos.set_from_frame_tab(the_robot->reply_package.arm.pf_def.arm_frame);
//		prev_pos = first_pos;
////
////		double tmp[3][3];
////		double tmp2[3];
////
////		first_pos.set_rotation_matrix(tmp);
////		first_pos.set_translation_vector(tmp2);
////
////		if((tmp2[0] == r) && (tmp2[1] == 0.0))
////			on_position = true;
////		else
////			on_position = false;
////
////		std::cout << "translation: x = " << tmp2[0] << " y = " << tmp2[1] << " z = " << tmp2[2] << std::endl;
////
////		std::cout << "rotation:  " << std::endl;
////		std::cout << tmp[0][0] << " " << tmp[0][1] << " " << tmp[0][2] << std::endl;
////		std::cout << tmp[1][0] << " " << tmp[1][1] << " " << tmp[1][2] << std::endl;
////        std::cout << tmp[2][0] << " " << tmp[2][1] << " " << tmp[2][2] << std::endl;
//	}
//
//	for (int i = 0; i < 6; i++) {
//		the_robot->ecp_command.instruction.arm.pf_def.behaviour[i] = lib::UNGUARDED_MOTION;
//	}
//	return true;
//}
//
//bool bclikeregions_gen::next_step(){
//
//	std::cout << "Stary generator" << std::endl;
//
//	the_robot->ecp_command.instruction.instruction_type = lib::SET_GET;
//
//	// get readings from all servos and aggregate
////	lib::Homog_matrix position;
////	position.set_from_frame_tab(the_robot->reply_package.arm.pf_def.arm_frame);
////	lib::Homog_matrix tmp;
////	countTranslationMatrix(tmp, 0.01, 0.0, 0.0);
////	countRotationMatrix(tmp, DTOR(5), 0.0, 0.0);
////	countRotationMatrix(tmp, DTOR(0), DTOR(0), DTOR(1));
////	countTranslationMatrix(tmp, 0.0, 0.01, 0.0);
////	position = position * tmp;
//
//	double prev[3];
//	double actual[3];
//
//	actual_pos.set_from_frame_tab(the_robot->reply_package.arm.pf_def.arm_frame);
//
////	if(!on_position){
////		double tmp[3];
////
////		actual_pos.set_translation_vector(tmp);
////
////		if((tmp[0] - r) < 0.01 ){//&& (tmp[1] == 0.0)){
////			on_position = true;
////			std::cout << "Na pozucji" << std::endl;
////		}else{
////
////			std::cout << "translation: x = " << tmp[0] << " y = " << tmp[1] << " z = " << tmp[2] << std::endl;
////
////			on_position = false;
////
////			countTranslationMatrix(next_pos, 0.01, 0.0, 0.0 );
////
////			next_pos = actual_pos * next_pos;
////		}
////	}else if(actual_pos == first_pos)
////		return false;
////	else{
////		prev_pos.get_translation_vector(prev);
////		actual_pos.get_translation_vector(actual);
////
////		angle = atan2(actual[1], actual[0]);
////
////		countTranslationMatrix(next_pos, r*cos(angle + DTOR(1)) - prev[0], r*sin(angle + DTOR(1)) - prev[1], 0.0 );
////
////		next_pos = actual_pos * next_pos;
////	}
//
////	if(!cubic_constr->is_position_ok(actual_pos))
////		return false
////		angle *= -1;
//
//	countTranslationMatrix(next_pos, 0.0, angle, 0.0);
//
//	next_pos = actual_pos * next_pos;
//
//	prev_pos = actual_pos;
//	next_pos.get_frame_tab(the_robot->ecp_command.instruction.arm.pf_def.arm_frame);
////	position.get_frame_tab(the_robot->ecp_command.instruction.arm.pf_def.arm_frame);
//
//	return true;
//}
//
//void bclikeregions_gen::countRotationMatrix(lib::Homog_matrix &mat, double x_angle, double y_angle, double z_angle){
//
//	double rot[3][3];
//	double trans[3];
//
//	memset(rot, 0, sizeof(double) * 9);
//	memset(trans, 0, sizeof(double) * 3);
//
//	for(int i = 0; i < 2; ++i)
//		rot[i][i] = 1;
//
//	if(x_angle > 0){
//		rot[1][1] = cos(x_angle);
//		rot[1][2] = -sin(x_angle);
//		rot[2][1] = sin(x_angle);
//		rot[2][2] = cos(x_angle);
//	}
//
//	if(y_angle > 0){
//		rot[0][0] = cos(y_angle);
//		rot[0][2] = sin(y_angle);
//		rot[2][0] = -sin(y_angle);
//		rot[2][2] = cos(y_angle);
//	}
//
//	if(z_angle > 0){
//		rot[0][0] = cos(z_angle);
//		rot[0][1] = -sin(z_angle);
//		rot[1][0] = sin(z_angle);
//		rot[1][1] = cos(z_angle);
//	}
//
//	mat.set_rotation_matrix(rot);
//	mat.set_translation_vector(trans);
//
//}
//
//void bclikeregions_gen::countTranslationMatrix(lib::Homog_matrix &mat, double x, double y, double z){
//
//	double rot[3][3];
//	double trans[3];
//
//	memset(rot, 0, sizeof(double) * 9);
//	memset(trans, 0, sizeof(double) * 3);
//
//	for(int i = 0; i < 2; ++i)
//		rot[i][i] = 1;
//
//	trans[0] = x;
//	trans[1] = y;
//	trans[2] = z;
//
//	mat.set_rotation_matrix(rot);
//	mat.set_translation_vector(trans);
//
//}
//
//
//}
//
//}
//
//}
//
//}
