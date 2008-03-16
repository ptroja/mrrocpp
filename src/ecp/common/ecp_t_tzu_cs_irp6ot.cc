#include <iostream.h>
#include <unistd.h>

#include "common/typedefs.h"
#include "common/impconst.h"
#include "common/com_buf.h"
#include <fstream>

#include "ecp_mp/ecp_mp_s_force.h"
#include "lib/srlib.h"
#include "ecp_mp/ecp_mp_t_rcsc.h"
#include "ecp_mp/ecp_mp_s_schunk.h"	

#include "ecp/irp6_on_track/ecp_local.h"
#include "ecp/common/ecp_g_tzu_cs.h"
#include "ecp/irp6_on_track/ecp_t_tzu_cs_irp6ot.h"
#include "lib/mathtr.h"
#include "ecp/common/ecp_g_smooth.h"

/** konstruktor **/
ecp_task_tzu_cs_irp6ot::ecp_task_tzu_cs_irp6ot(configurator &_config) : ecp_task(_config)
{
	sg = NULL;
	befg = NULL;
};

/** destruktor **/
ecp_task_tzu_cs_irp6ot::~ecp_task_tzu_cs_irp6ot()
{
};


// methods for ECP template to redefine in concrete classes
void ecp_task_tzu_cs_irp6ot::task_initialization(void) 
{
cout<<"inicjalizacja robota"<<endl;
	ecp_m_robot = new ecp_irp6_on_track_robot (*this);
	//cout<<"inicjalizacja skonczona"<<endl;
	sg = new ecp_smooth_generator (*this, true, true);
	befg = new bias_edp_force_generator(*this);
	f = new tzu_simple_generator(*this,8);	
	//cout<<"wczytanie trajektorii"<<endl;	
	//sg->load_file_with_path("../trj/tzu.trj");
	//cout<<"ustawianie generatora skonczone"<<endl;	
	short use_force_sensor = config.return_int_value("use_force_sensor");
    	if (use_force_sensor == 1)
	{
		cout<<"uzywamy czujnika sily"<<endl;
        	sr_ecp_msg->message("Using force sensor for move control");
         	// Stworzenie obiektu czujnik.
		// ini_con->create_vsp ("[vsp_fs]");
		sensor_m[SENSOR_FORCE_ON_TRACK] = new ecp_mp_force_sensor(SENSOR_FORCE_ON_TRACK, "[vsp_fs]", *this);
		// Konfiguracja czujnika.
		sensor_m[SENSOR_FORCE_ON_TRACK]->configure_sensor();
         	// Stworzenie listy czujnikow -> glowa = (czujnik sily).
        	//fctg->sensor_m[SENSOR_FORCE_ON_TRACK] = sensor_m[SENSOR_FORCE_ON_TRACK];
         	// Odczyt wielkosci niebezpiecznej sily z pliku INI.
         	//fctg->set_dangerous_force();
		cout<<"czujnik skonfigurowany"<<endl;
	}
	else
	{
		cout<<"nie uzywamy czujnika sily"<<endl;
          sr_ecp_msg->message("Not using force sensor for move control");
          // Pusta lista czujnikow.
          //fctg->sensor_m.clear();
          sensor_m.clear();
          // Pusty czujnik.
	};  // end: else
	sr_ecp_msg->message("ECP loaded");
};

void ecp_task_tzu_cs_irp6ot::main_task_algorithm(void)
{
	sr_ecp_msg->message("ECP cs irp6ot  - pushj start in tzu");
	ecp_wait_for_start();
	
	/* stworzenie generatora ruchu */
	// sprawdzic czy drugi argument jest rzeczywiscie potrzebny
	//tzu_simple_generator sg(*this, 8);
	// sprawdzic zmienna sensor_m
	//sg.sensor_m = sensor_m;
	
	//trzeba jeszcze stworzy generatory ktore zajma sie odczytaniem sily
	while(true)			
	{ 
		// sprawdzic co robi metoda move znajdujaca sie pewnie w ecp_task
		sg->load_file_with_path("../trj/tzu/tzu_1.trj");
		//sg->Move ();
		cout<<"Piierwsza czesc ruchu skonczona"<<endl;
		sr_ecp_msg->message("FORCE SENSOR BIAS");
		if(befg != NULL)
			befg->Move();
		cout<<"Biasowanie czujnika sily dokonane..."<<endl;
		//sleep(5);
		sg->load_file_with_path("../trj/tzu/tzu_2.trj");
		//sg->Move ();
		sg->load_file_with_path("../trj/tzu/tzu_3.trj");
		cout<<"Druga czesc ruchu skonczona"<<endl;
		//sg->Move ();
		cout<<"wykonane move"<<endl;
		cout<<"Trzecia czesc ruchu skonczona"<<endl;
		cout<<"wait for stop\n"<<endl;
		
		//cout<<"mamy: "<<sensor_m<<endl;
		if(sensor_m.begin()->second != NULL)
			cout<<"korwa rozne: "<<endl;
		else
			cout<<"dupa zbita"<<endl;
//		f->Move();
		cout<<"force_0: "<<sensor_m.begin()->second->image.force.rez[0]<<endl;
		cout<<"force_1: "<<sensor_m.begin()->second->image.force.rez[1]<<endl;
		cout<<"force_2: "<<sensor_m.begin()->second->image.force.rez[2]<<endl;

		ecp_wait_for_stop();
		break;
	}
	cout<<"end\n"<<endl;
};

// sprawdzic co robi ta metoda, gdzie, w jakich przypadkach jest uzywana
ecp_task* return_created_ecp_task (configurator &_config)
{
	return new ecp_task_tzu_cs_irp6ot(_config);
};

