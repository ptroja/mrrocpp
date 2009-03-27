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
        : ecp_generator (_ecp_task)
{
	//tool to plate frame initialization, all in rad and meters
	plate_to_tool[0]=0; plate_to_tool[1]=0; plate_to_tool[2]=1; plate_to_tool[3]=0.001;
	plate_to_tool[4]=-1; plate_to_tool[5]=0; plate_to_tool[6]=0; plate_to_tool[7]=0.000;
	plate_to_tool[8]=0; plate_to_tool[9]=-1; plate_to_tool[10]=0; plate_to_tool[11]=0.04;
	plate_to_tool[12]=0; plate_to_tool[13]=0; plate_to_tool[14]=0; plate_to_tool[15]=1;

	//spots in plate coordinates
	vec_1[0]=-0.035; vec_1[1]=0.035; vec_1[2]=0.001; vec_1[3]=1.;
	vec_2[0]=0.035; vec_2[1]=0.035; vec_2[2]=0.001; vec_2[3]=1.;
	vec_3[0]=0.035; vec_3[1]=-0.035; vec_3[2]=0.001; vec_3[3]=1.;
	vec_4[0]=-0.035; vec_4[1]=-0.035; vec_4[2]=0.001; vec_4[3]=1.;
}

bool ecp_spots_generator::first_step()
{
	sensor = (ecp_mp_cvfradia_sensor *)sensor_m[SENSOR_CVFRADIA];

	//proste zadanie kinematyki
	the_robot->EDP_data.instruction_type = GET;
	the_robot->EDP_data.get_type = ARM_DV;
	the_robot->EDP_data.get_arm_type = FRAME;

	//zadanie zrobienia zdjec od fraidii
	sensor->to_vsp.command = 38;
	sensor->to_vsp.i_code = VSP_INITIATE_READING;

	iter = 0;

	return true;
}

bool ecp_spots_generator::next_step()
{
	if(iter == 0) //first time next_step
	{
	    get_frame();
	    iter++;
	    return true;
	}

	// jesli zdjecia niegotowe, return true
    sensor->to_vsp.command = 0;


	//printf("%d\n", sensor->from_vsp.comm_image.sensor_union.sp_r.pic_count);

    if(sensor->from_vsp.vsp_report == VSP_REPLY_OK && sensor->from_vsp.comm_image.sensor_union.sp_r.pic_count <= -2)
		//this position failed
		return false;
    else if(sensor->from_vsp.vsp_report != VSP_REPLY_OK || sensor->from_vsp.comm_image.sensor_union.sp_r.pic_count <= 0)
		return true;

		//teraz zabawa z wynikami


		get_pic();
		save_position();


		return false;
}

void ecp_spots_generator::get_frame()
{
	for(int i=0; i<3; i++)
	{
		for(int j=0; j<4; j++)
			tool_to_ground[4*i+j] = the_robot->EDP_data.current_arm_frame[i][j];
	}

	for(int i=12; i<15; i++)
		tool_to_ground[i] = 0;
	tool_to_ground[15] = 1;
}

void ecp_spots_generator::get_pic()
{
	calib_data.sensor_union.sp_r.dz = sensor->from_vsp.comm_image.sensor_union.sp_r.dz;
	calib_data.sensor_union.sp_r.pic_count = sensor->from_vsp.comm_image.sensor_union.sp_r.pic_count;
	for (int i=0; i<4; i++)
	{
		calib_data.sensor_union.sp_r.x[i] = sensor->from_vsp.comm_image.sensor_union.sp_r.x[i];
		calib_data.sensor_union.sp_r.y[i] = sensor->from_vsp.comm_image.sensor_union.sp_r.y[i];
		calib_data.sensor_union.sp_r.z[i] = sensor->from_vsp.comm_image.sensor_union.sp_r.z[i];
	}
}

void ecp_spots_generator::save_position()
{
	Matrix4x4 T; // tego szukamy
	Matrix4x4 D;
	double d_matrix[16];
	double t_matrix[16];
	double * x;
	double c[4];

	//Matrix4x4 t_t_g(ground_to_tool, TYPE_T);
	//double * tool_to_ground = t_t_g.getinv();

	//Matrix4x4 p_t_t(tool_to_plate, TYPE_T);
	//double * plate_to_tool = p_t_t.getinv();

	Matrix4x4 p_t_g(tool_to_ground, TYPE_T);
	p_t_g.rproduct4x4(plate_to_tool);
	//double * plate_to_ground = p_t_g.getA();

FILE *fd = fopen("../msr/kalibracja.txt", "a+");
	for(int i=0; i<4; i++)
	{
		double vec_plate[4];
		double x, y;

		switch(i)
		{
		  case 0:
			  for(int i=0; i<4; i++)
				  vec_plate[i] = vec_1[i];
			  break;
		  case 1:
			  for(int i=0; i<4; i++)
				  vec_plate[i] = vec_2[i];
			  break;
		  case 2:
			  for(int i=0; i<4; i++)
				  vec_plate[i] = vec_3[i];
			  break;
		  case 3:
			  for(int i=0; i<4; i++)
				  vec_plate[i] = vec_4[i];
			  break;
		  default:
			  for(int i=0; i<4; i++)
				  vec_plate[i] = 0.;
			  break;
		}
		p_t_g.product4x1(vec_plate); // vec_plate := vec_ground

//FILE *fd1 = fopen("../msr/temp.txt", "a+");
//for(int it = 0; it<4; it++)
//		fprintf(fd1, "%f ", tool_to_ground[4*i+it]);
//for(int it=0; it<4; it++)
//	fprintf(fd1, "%f ",vec_plate[it]);
//fprintf(fd1, "\n");
//fclose(fd1);

for(int it=0; it<4; it++)
	fprintf(fd, "%f ",vec_plate[it]);
fprintf(fd, "%f %f %f 1.0 %f\n", 0.01*calib_data.sensor_union.sp_r.x[i], 0.01*calib_data.sensor_union.sp_r.y[i], 0.01*calib_data.sensor_union.sp_r.z[i], calib_data.sensor_union.sp_r.dz);
		//double vec_cam[3];
		//vec_cam[0] = calib_data.sensor_union.sp_r.x[i];
		//vec_cam[1] = calib_data.sensor_union.sp_r.y[i];
		//vec_cam[2] = calib_data.sensor_union.sp_r.z[i];

		for(int j=0; j<4; j++)
		    d_matrix[4*i+j] = vec_plate[j];
	}

	D.setA(d_matrix, TYPE_D);



//	for (int i=0; i<4; i++)
//	{
//		for(int j=0; j<4; j++)
//			fprintf(fd, "%f  ", d_matrix[4*i+j]);
//		fprintf(fd, "\n");
//	}fprintf(fd, "\n");

	//after that vec_plate is spot postion in ground coordinates
	//computation described in my thesis
	//D matrix's rows are spots' x,y,zs and 1s
	//c vectors are spots' xs then ys and finally zs
	//x vectors are rows of T

	for (int i=0; i<3; i++)
	{
		switch (i)
		{
		  case 0:
			for (int k=0; k<4; k++)
				c[k] = 0.01*calib_data.sensor_union.sp_r.x[k];
			break;
		  case 1:
			for (int k=0; k<4; k++)
				c[k] = 0.01*calib_data.sensor_union.sp_r.y[k];
			break;
		  case 2:
			for (int k=0; k<4; k++)
				c[k] = 0.01*calib_data.sensor_union.sp_r.z[k];
			break;
		  default:
			  break;
		}
//		D.setb(c);
//		D.solveAxb4x4();
//		x = D.getx();
//
//		for (int j=0; j<4; j++)
//			t_matrix[4*j+i] = x[j];
	}
//	T.setA(t_matrix, TYPE_T);

//	for (int i=0; i<4; i++)
//	{
//		for(int j=0; j<4; j++)
//			printf("%.3f  ", t_matrix[4*i+j]);
//		printf("\n");
//	}printf("\n\n");
fclose(fd);
}

