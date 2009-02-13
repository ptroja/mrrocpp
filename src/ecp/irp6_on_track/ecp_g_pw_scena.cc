#include "ecp/irp6_on_track/ecp_g_pw_scena.h"
#include <iostream>
ecp_g_pw_scena::ecp_g_pw_scena(ecp_task& _ecp_task) :
	ecp_generator(_ecp_task) {

	recognized = false;

	down = false;

	xdir = 0;
	ydir = 0;
}

bool ecp_g_pw_scena::first_step() {
	the_robot->EDP_data.instruction_type = GET;
	the_robot->EDP_data.get_type = ARM_DV;
	the_robot->EDP_data.set_type = ARM_DV;
	the_robot->EDP_data.set_arm_type = XYZ_ANGLE_AXIS;
	the_robot->EDP_data.get_arm_type = XYZ_ANGLE_AXIS;
	the_robot->EDP_data.motion_type = ABSOLUTE;
	the_robot->EDP_data.next_interpolation_type = MIM;
	the_robot->EDP_data.motion_steps = 8;
	the_robot->EDP_data.value_in_step_no = 6;

	recognized = false;

	down = false;

	xdir = 0;
	ydir = 0;

	step_no = 1;
	//    for(int i=0;i<8;i++)
	//    	std::cout<<next_position[i]<<std::endl;
	return true;
}


double* ecp_g_pw_scena::get_current_pose(){

	double pos[6];
	memcpy(pos,
			the_robot->EDP_data.current_XYZ_AA_arm_coordinates, 6
					* sizeof(double));

	return pos;
}



bool ecp_g_pw_scena::next_step() {

	step_no++;

	//Read current position.
	if (step_no == 2) {
		memcpy(next_position,
				the_robot->EDP_data.current_XYZ_AA_arm_coordinates, 6
						* sizeof(double));
		next_position[6] = the_robot->EDP_data.current_gripper_coordinate;

		recognized =  false;
	}


	double xdelta = 0;
	double ydelta = 0;

	//Pobieram odczyty z fradii.
	//sensor_m[SENSOR_CVFRADIA]->get_reading();

	if (!(sensor_m[SENSOR_CVFRADIA]->from_vsp.comm_image.sensor_union.deviation.frame_number
			== 0) && !recognized) {

		double
				xdev =
						sensor_m[SENSOR_CVFRADIA]->from_vsp.comm_image.sensor_union.deviation.x;
		double
				ydev =
						sensor_m[SENSOR_CVFRADIA]->from_vsp.comm_image.sensor_union.deviation.y;

		std::cout << "Obraz st1. xdev: " << xdev << ", ydev: " << ydev << std::endl;

		double dir;
		if (ydev == 0) {
			xdir = 0;
			ydir = 0.005;
		} else if (xdev == 0) {
			xdir = 0.005;
			ydir = 0;
		} else {
			//Przydalaby sie macierz relacji kamera chwytak.
			double rad = atan2(-xdev, -ydev);

			xdir = cos(rad) * 0.005;
			ydir = sin(rad) * 0.005;
		}

		std::cout << "Recognized1. xdir: " << xdir << ", ydir: " << ydir
				<< ", tang: " << dir << std::endl;

		recognized = true;

	} else if (!(sensor_m[SENSOR_CVFRADIA]->from_vsp.comm_image.sensor_union.deviation.frame_number
			== 0) && recognized ) {

		double
				xdev =
						sensor_m[SENSOR_CVFRADIA]->from_vsp.comm_image.sensor_union.deviation.x;
		double
				ydev =
						sensor_m[SENSOR_CVFRADIA]->from_vsp.comm_image.sensor_union.deviation.y;

		//Warunek stopu.
		if (fabs(xdev) < 10 && fabs(ydev) < 10)
			return false;


		double dir;
		if (ydev == 0) {
			xdir = 0;
			ydir = (xdev < 0 ? 1:-1)*0.005;
		} else if (xdev == 0) {
			xdir = (ydev < 0 ? 1:-1)*0.005;
			ydir = 0;
		} else {
			//Przydalaby sie macierz relacji kamera chwytak.
			double rad = atan2(-xdev, -ydev);

			xdir = cos(rad) * 0.005;
			ydir = sin(rad) * 0.005;
		}

		next_position[0] += xdir; //x
		next_position[1] += ydir; //y

		std::cout << "Recognized2. xdir: " << xdir << ", ydir: " << ydir
				<< ", tang: " << dir << std::endl;
	}



	double time = 0.3; //Czas ruchu.
	the_robot->EDP_data.instruction_type = SET;
	the_robot->EDP_data.set_type = ARM_DV; // ARM
	the_robot->EDP_data.set_arm_type = XYZ_ANGLE_AXIS;
	the_robot->EDP_data.motion_type = ABSOLUTE;
	the_robot->EDP_data.next_interpolation_type = MIM;
	the_robot->EDP_data.motion_steps = (WORD) ceil(time / STEP);
	the_robot->EDP_data.value_in_step_no = the_robot->EDP_data.motion_steps;

	memcpy(the_robot->EDP_data.next_XYZ_AA_arm_coordinates, next_position, 6
			* sizeof(double));
	the_robot->EDP_data.next_gripper_coordinate = next_position[6];

	return true;
}
