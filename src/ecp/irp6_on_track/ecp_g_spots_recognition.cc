/*
 * ecp_g_spots_recognition.cc
 *
 *  Created on: Aug 26, 2008
 *      Author: ghard
 */

#include "ecp/irp6_on_track/ecp_g_spots_recognition.h"
#include <unistd.h>


ecp_spots_generator::ecp_spots_generator (ecp_task& _ecp_task)
        : ecp_smooth_generator (_ecp_task, true, true)
{

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
		sleep(20);


		return false;
	}
}
