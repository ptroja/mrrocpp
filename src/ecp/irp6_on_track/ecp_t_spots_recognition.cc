/*!
 * \file ecp_t_spots_recognition.cc
 * \brief Piotr Sakowicz's methods responsible
 * for robot moving while external calibration.
 * - methods definitions
 * \author vented.baffle
 * \date 21.08.2008
 */

#include "ecp/irp6_on_track/ecp_t_spots_recognition.h"

#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include <sys/types.h>
#include <sys/stat.h>

//own libraries
//Constructors
ecp_t_spots_recognition::ecp_t_spots_recognition(configurator &_config): ecp_task(_config)
{
    katalog_traj = "../trj/spots/standard/traj";
    katalog_dump = "~/mrroc_calib";
    mkdir(katalog_dump, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    remove("../msr/kalibracja.txt");
}

ecp_t_spots_recognition::~ecp_t_spots_recognition()
{

}

// methods for ECP template to redefine in concrete classes
void ecp_t_spots_recognition::task_initialization(void)
{

	// Create cvFraDIA sensor - for testing purposes.
	sensor_m[SENSOR_CVFRADIA] = new ecp_mp_cvfradia_sensor(SENSOR_CVFRADIA, "[vsp_cvfradia]", *this, sizeof(sensor_image_t::sensor_union_t::sp_r_t));
	// Configure sensor.
	sensor_m[SENSOR_CVFRADIA]->configure_sensor();

// Create an adequate robot. - depending on the ini section name.
// Note - only on track working
if (strcmp(config.section_name, "[ecp_irp6_on_track]") == 0)
{
    ecp_m_robot = new ecp_irp6_on_track_robot (*this);
    sr_ecp_msg->message("IRp6 on Track loaded");
}
/*else if (strcmp(config.section_name, "[ecp_irp6_postument]") == 0)
{
    ecp_m_robot = new ecp_irp6_postument_robot (*this);
    sr_ecp_msg->message("IRp6 Postument loaded");
}*/
	// Create spots generator and pass sensor to it.
	generator = new ecp_spots_generator(*this);
 	generator->sensor_m = sensor_m;

	// Create smooth generator.
	smooth = new ecp_smooth_generator(*this, true, false);

	//sleep_g = new ecp_sleep_generator(*this);

}

void ecp_t_spots_recognition::main_task_algorithm(void)
{
	char traj[36];

    for(int i=0; i<=15; i++)
    {
    	/*!
    	 * smooth generator odczytuje trajektorie z pliku
    	 * po czym przesuwa sie do odpowiedniej pozycji
    	 * jest 15 trajektorii + poczatkowa + kocowa (poza petla)
    	 */
    	sprintf(traj, "%s%.2d.trj", katalog_traj, i);
    	//printf("teraz iteracja %d", i);
    	smooth->load_file_with_path(traj);
	    smooth->Move();

	    /*!
	     * nastepnie nalezy odczekac ok 1s, zanim mozna rozpoczac sesje zdjeciowa
	     */
	    sleep(2);
	    //sleep_g->init_time(2);
	    //sleep_g->Move();

	    /*!
	     * zrob zdjecia, dokonaj obliczen
	     */
	    generator->Move();

	    //printf(" ... koniec\n");
    }

    sprintf(traj, "%s%.2d.trj", katalog_traj, 99);
	smooth->load_file_with_path(traj);
    smooth->Move();
}

ecp_task* return_created_ecp_task(configurator &_config)
{
	return new ecp_t_spots_recognition(_config);
}
