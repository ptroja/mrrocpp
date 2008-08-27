/*
 * ecp_g_spots_recognition.cc
 *
 *  Created on: Aug 26, 2008
 *      Author: ghard
 */

#include "ecp/irp6_on_track/ecp_g_spots_recognition.h"
#include "ecp/common/ecp_matrix4x4.h"
#include <unistd.h>


ecp_spots_generator::ecp_spots_generator (ecp_task& _ecp_task)
        : ecp_smooth_generator (_ecp_task, true, true)
{
	//tool to plate frame initialization
	plate_to_tool[0]=0; plate_to_tool[1]=0; plate_to_tool[2]=1; plate_to_tool[3]=0.1;
	plate_to_tool[4]=-1; plate_to_tool[5]=0; plate_to_tool[6]=0; plate_to_tool[7]=0;
	plate_to_tool[8]=0; plate_to_tool[9]=-1; plate_to_tool[10]=0; plate_to_tool[11]=4;
	plate_to_tool[12]=0; plate_to_tool[13]=0; plate_to_tool[14]=0; plate_to_tool[15]=1;

	//spots in plate coordinates
	vec_1[0]=0; vec_1[1]=0; vec_1[2]=0;
	vec_2[0]=0; vec_2[1]=0; vec_2[2]=0;
	vec_3[0]=0; vec_3[1]=0; vec_3[2]=0;
	vec_4[0]=0; vec_4[1]=0; vec_4[2]=0;
}

bool ecp_spots_generator::first_step()
{
	sensor = (ecp_mp_cvfradia_sensor *)sensor_m[SENSOR_CVFRADIA];


//	sensor->to_vsp.command = 38;
//	sleep(5);
//	sensor->to_vsp.command = 0;

	return ecp_smooth_generator::first_step();
}

bool ecp_spots_generator::next_step()
{
	//czy ruch w sensie smooth generatora sie zakonczyl, czy jeszcze trwa
	bool czy_ruch = ecp_smooth_generator::next_step();

	//jesli nie, po prostu go wykonaj.
	//if(czy_ruch)
	{
		//return czy_ruch;
	}
	//jesli sie zakonczyl, trzeba zrobic zdjecia
	//else
	{
		//zadanie zrobienia zdjec od fraidii
		comm_struct.command = 38;
		comm_struct.i_code = VSP_INITIATE_READING;
		sensor->send_reading(comm_struct);

		int i=0; //liczba iteracji, po ktorej 38 -> 0
		do
		{
			if(i<10)
			    sensor->to_vsp.command = 38;
			else
				sensor->to_vsp.command = 0;
			//sensor->send_reading(comm_struct);
			sensor->get_reading();
			i++;
		}
		while(sensor->from_vsp.vsp_report == VSP_READING_NOT_READY);
		sensor->to_vsp.command = 0;

		//teraz zabawa z wynikami
		//sleep(20);


		get_frame();
		get_pic();
		save_position();


		return false;
	}
}

void ecp_spots_generator::get_frame()
{
	the_robot->EDP_data.instruction_type = GET;
	the_robot->EDP_data.get_type = ARM_DV;
	the_robot->EDP_data.get_arm_type = FRAME;

	for(int i=0; i<3; i++)
		for(int j=0; j<4; j++)
			tool_to_ground[4*i+j] = the_robot->EDP_data.current_arm_frame[i][j];

	for(int i=12; i<15; i++)
		tool_to_ground[i] = 0;
	tool_to_ground[15] = 1;

}

void ecp_spots_generator::get_pic()
{
	calib_data.sp_r.dz = sensor->image.sp_r.dz;
	calib_data.sp_r.pic_count = sensor->image.sp_r.pic_count;
	for (int i=0; i<4; i++)
	{
		calib_data.sp_r.x[i] = sensor->image.sp_r.x[i];
		calib_data.sp_r.y[i] = sensor->image.sp_r.y[i];
		calib_data.sp_r.z[i] = sensor->image.sp_r.z[i];
	}
}

void ecp_spots_generator::save_position()
{
	Matrix4x4 m; // tego szukamy

	//Matrix4x4 t_t_g(ground_to_tool, TYPE_T);
	//double * tool_to_ground = t_t_g.getinv();

	//Matrix4x4 p_t_t(tool_to_plate, TYPE_T);
	//double * plate_to_tool = p_t_t.getinv();

	Matrix4x4 p_t_g(tool_to_ground, TYPE_T);
	p_t_g.rproduct4x4(plate_to_tool);
	//double * plate_to_ground = p_t_g.getA();

	for(int i=0; i<4; i++)
	{
		double * vec_plate;
		double x, y;

		switch(i)
		{
		  case 0:
			  vec_plate = vec_1;
			  x=-3.5;
			  y=-3.5;
			  break;
		  case 1:
			  vec_plate = vec_2;
			  x=-3.5;
			  y=3.5;
			  break;
		  case 2:
			  vec_plate = vec_3;
			  x=3.5;
			  y=3.5;
			  break;
		  case 3:
			  vec_plate = vec_4;
			  x=3.5;
			  y=-3.5;
			  break;
		  default:
			  vec_plate = 0;
			  break;
		}

		p_t_g.product4x1(vec_plate); // vec_plate := vec_ground

		double vec_cam[3];
		vec_cam[0] = calib_data.sp_r.x[i];
		vec_cam[1] = calib_data.sp_r.y[i];
		vec_cam[2] = calib_data.sp_r.z[i];
	}
}

