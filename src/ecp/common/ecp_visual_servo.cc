///////////////////////////////////////////////////////////
//  ecp_visual_servo.cpp
//  Implementation of the Class ecp_visual_servo
//  Created on:      04-sie-2008 14:21:20
//  Original author: tkornuta
///////////////////////////////////////////////////////////

/*!
 * \file ecp_visual_servo.cc
 * \brief Abstract class as a pattern for implementing any visual servo.
 * - methods definition
 * \author tkornuta/mstaniak
 * \date 04.08.2008
 */

#include "ecp/common/ecp_visual_servo.h"
  
ecp_visual_servo::ecp_visual_servo(ecp_task& _ecp_task, int step) : ecp_generator(_ecp_task){

}



ecp_visual_servo::~ecp_visual_servo(){

}


void ecp_visual_servo::retrieve_parameters(){

}


bool ecp_visual_servo::next_step(void){

	next_step_without_constraints();
	entertain_constraints();
	
	return true;
}


void ecp_visual_servo::set_constraints(){


}


void ecp_visual_servo::get_constraints(){

}


void ecp_visual_servo::set_entities(){

}


void ecp_visual_servo::get_entities(){

}


void ecp_visual_servo::set_opartions(){

}


void ecp_visual_servo::get_operations(){

}
