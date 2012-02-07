/*
 * AndroidTeach.cpp
 *
 *  Created on: Nov 20, 2011
 *      Author: hh7
 */



#include <cstring>

#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"
#include <fstream>

#include <sys/stat.h>

#include "base/lib/sr/srlib.h"
#include "application/android_teach/sensor/EcpMpAndroid.h"

#include "application/android_teach/AndroidTeach.h"
#include "base/ecp/ecp_task.h"
#include "robot/irp6ot_m/ecp_r_irp6ot_m.h"

#include "base/lib/mrmath/mrmath.h"
#include "base/lib/messip/messip_dataport.h"






namespace mrrocpp {
namespace ecp {
namespace irp6ot_m {
namespace task {



AndroidTeach::~AndroidTeach()
{
	// TODO Auto-generated destructor stub
}




AndroidTeach::AndroidTeach(lib::configurator &_config) :
	common::task::task(_config)
{
	ecp_m_robot = (boost::shared_ptr<robot_t>) new robot(*this);
	trajectory.count = trajectory.position = 0;
	trajectory.head = trajectory.tail = trajectory.current = NULL;

	pose_spec = lib::ECP_JOINT;
	type = "JOINT";
	axis_num = 7;

//	REGISTER_SYSTEM_ERROR(NetworkException, "cannot");


	gg = new common::generator::get_position(*this, lib::ECP_JOINT, 7);

	gg->Move();


	if (!gg)
		perror("gg jest zle");

	androidState.readConfig(_config, gg->get_position_vector());

	//create Android virtual sensor object
	sensor_m[ecp_mp::sensor::SENSOR_ANDROID]
			= new ecp_mp::sensor::EcpMpAndroid(ecp_mp::sensor::SENSOR_ANDROID, "[vsp_android]", *this->sr_ecp_msg, this->config, &this->androidState, this->gg);


	//configure the sensor
//	sensor_m[ecp_mp::sensor::SENSOR_ANDROID]->configure_sensor();
}

int AndroidTeach::load_trajectory()
{
	if (chdir(path) != 0)
	{
		perror(path);
		throw ecp_mp::sensor::android_teach::NetworkException();//common::robot::ECP_error(lib::NON_FATAL_ERROR, NON_EXISTENT_DIRECTORY);
	}

	std::ifstream from_file(filename); // otworz plik do zapisu

	if (!from_file)
	{
		perror(filename);
		throw ecp_mp::sensor::android_teach::NetworkException();//common::robot::ECP_error(lib::NON_FATAL_ERROR, NON_EXISTENT_FILE);
	}

	node* current = NULL;
	trajectory.position = 0;
	trajectory.count = 0;
	int count = 0;

	struct stat filestatus;
	stat(filename, &filestatus);

	if(filestatus.st_size)
	{
		from_file >> type;
		from_file >> count;
		from_file >> mode;

		if(!strcmp(type.c_str(),"JOINT"))
		{
			pose_spec = lib::ECP_JOINT;
			axis_num = 7;
		}
		else
		{
			pose_spec = lib::ECP_XYZ_ANGLE_AXIS;
			axis_num = 6;
		}

		coordinates = std::vector<double>(axis_num);

		do
		{
			getline(from_file, velocity);
		} while (!velocity.length() || from_file.eof());

		do
		{
			getline(from_file, acceleration);
		} while (!acceleration.length() || from_file.eof());


		while (trajectory.count < count && !from_file.eof())
		{
			++trajectory.count;
			if (current)
			{
				current->next = new node();
				current->next->prev = current;
				current = current->next;
			}
			else
			{
				current = new node();
				trajectory.head = current;
			}

			current->position = std::vector<double>(axis_num);
			current->id = trajectory.count;

			trajectory.tail = current;
			for(int i = 0; i < axis_num; ++i)
			{
				from_file >> current->position[i];
			}

			sprintf(buffer, "Loaded %s %d: %.4f %.4f %.4f %.4f %.4f %.4f %.4f", type.c_str(), trajectory.count, current->position[0], current->position[1], current->position[2], current->position[3], current->position[4], current->position[5], axis_num > 6 ? current->position[6] : 0);
			sr_ecp_msg->message(buffer);
		}

		trajectory.current = trajectory.head;
		trajectory.position = 1;
	}

	return 0;
}

bool AndroidTeach::get_filenames(void)
{
//begin by OL
//	return false;

        char newFilename[20], newPath[80];
        sprintf(newFilename,"test_traj.trj");
        sprintf(newPath,"/home/oleszczy/workspace/");

        strncpy(path, newPath, 79);
        strncpy(filename, newFilename, 19);
//
//    char buffer[100];
//
//    sprintf(buffer,"traj_filename");
//    sprintf(newFilename,"%s",config.value<std::string>(buffer));
//
//    sprintf(buffer,"traj_path");
//    sprintf(newPath,"%s",config.value<std::string>(buffer));
//
//    strncpy(path, newPath, 79);
//    strncpy(filename, newFilename, 19);


	return true;

//// end by OL
//
//        lib::ECP_message ecp_to_ui_msg; // Przesylka z ECP do UI
//        lib::UI_reply ui_to_ecp_rep; // Odpowiedz UI do ECP
//        uint64_t e; // Kod bledu systemowego
//
////	ecp_to_ui_msg.ecp_message = lib::SAVE_FILE; // Polecenie wprowadzenia nazwy pliku
////	strcpy(ecp_to_ui_msg.string, "*.trj"); // Wzorzec nazwy pliku
//
//        // if ( Send (UI_pid, &ecp_to_ui_msg, &ui_to_ecp_rep, sizeof(lib::ECP_message), sizeof(lib::UI_reply)) == -1) {
//
//        if(messip::port_send(this->UI_fd, 0, 0, ecp_to_ui_msg, ui_to_ecp_rep) < 0) // by Y&W
//        {
//                e = errno;
//                perror("ecp: Send() to UI failed");
//                sr_ecp_msg->message(lib::SYSTEM_ERROR, e, "ecp: Send() to UI failed");
//                throw common::robot::ECP_error(lib::SYSTEM_ERROR, 0);
//        }
//        else  //begin by OL
//        {
//                perror("ecp: Send() to UI succed");
//        }	// end by OL
//
//        if (ui_to_ecp_rep.reply == lib::QUIT) // Nie wybrano nazwy pliku lub zrezygnowano z zapisu
//        {
//                perror("ecp: Send() to UI reply QUIT");  // by OL
//                return false;
//        }
//
//        strncpy(path, ui_to_ecp_rep.path, 79);
//        strncpy(filename, ui_to_ecp_rep.filename, 19);

	return true;
}

void AndroidTeach::save_trajectory(void)
{
//	if (chdir(path) != 0)
//	{
//		perror(path);
//		throw common::robot::ECP_error(lib::NON_FATAL_ERROR, NON_EXISTENT_DIRECTORY);
//	}
//
	std::ofstream to_file(filename); // otworz plik do zapisu
//	if (!to_file)
//	{
//		perror(filename);
//		throw common::robot::ECP_error(lib::NON_FATAL_ERROR, NON_EXISTENT_FILE);
//	}

	node* current = trajectory.head;

	to_file << type << '\n';

	to_file << trajectory.count << '\n';

	to_file << "ABSOLUTE" << '\n' << '\n';

	to_file << velocity << '\n' << acceleration << '\n';

	while (current)
	{
		for(int i = 0; i < axis_num; ++i)
		{
			to_file << current->position[i] << ' ';
		}

		to_file << '\n';

		current = current->next;
	}

	sprintf(buffer, "Trajectory saved to %s/%s", path, filename);
	sr_ecp_msg->message(buffer);
}



void AndroidTeach::print_trajectory(void)
{
	node* current = trajectory.head;

	sr_ecp_msg->message("=== Trajektoria ===");

	while (current)
	{
		sprintf(buffer, "Pozycja %s %d: %.4f %.4f %.4f %.4f %.4f %.4f %.4f", type.c_str(), current->id, current->position[0], current->position[1], current->position[2], current->position[3], current->position[4], current->position[5], axis_num > 6 ? current->position[6] : 0);
		sr_ecp_msg->message(buffer);
		current = current->next;
	}

	sr_ecp_msg->message("=== Trajektoria - koniec ===");
}

void AndroidTeach::move_to_next(void)
{
//	buttonsPressed.left = 0;
	if (trajectory.position < trajectory.count)
	{
		++trajectory.position;
		trajectory.current = trajectory.current->next;
		move_to_current();
	}
}

void AndroidTeach::move_to_prev(void)
{
//	buttonsPressed.right = 0;
	if (trajectory.position > 1)
	{
		--trajectory.position;
		trajectory.current = trajectory.current->prev;
		move_to_current();
	}
}

//void AndroidTeach::handleHome(void)
//{
//	buttonsPressed.buttonHome = 0;
//	if (trajectory.count && trajectory.position > 0)
//	{
//		trajectory.current->position = gg->get_position_vector();
//
//		sprintf(buffer, "Changed %s %d: %.4f %.4f %.4f %.4f %.4f %.4f %.4f", type.c_str(), trajectory.current->id, trajectory.current->position[0], trajectory.current->position[1], trajectory.current->position[2], trajectory.current->position[3], trajectory.current->position[4], trajectory.current->position[5], axis_num > 6 ? trajectory.current->position[6] : 0);
//		sr_ecp_msg->message(buffer);
//	}
//
//	print_trajectory();
//}
//
//void AndroidTeach::handle12(void)
//{
//	if (buttonsPressed.button1)
//	{
//		buttonsPressed.button1 = 0;
//		gen = ++gen % 3;
//	}
//	else if (buttonsPressed.button2)
//	{
//		buttonsPressed.button2 = 0;
//		gen = --gen % 3;
//		if (gen < 0)
//		{
//			gen = 2;
//		}
//	}
//
//	message.led_change = true;
//	message.rumble = false;
//	switch (gen)
//	{
//		case 0:
//			g = ag;
//			message.led_status = 0x1;
//			break;
//		case 1:
//			g = rg;
//			message.led_status = 0x2;
//			break;
//		case 2:
//			g = jg;
//			message.led_status = 0x4;
//			break;
//	}
//	((ecp_mp::sensor::wiimote*) sensor_m[ecp_mp::sensor::SENSOR_WIIMOTE])->send_reading(message);
//}
//

void AndroidTeach::handleMinus(void)
{
//	buttonsPressed.buttonMinus = 0;
	if (trajectory.position > 0)
	{
		node* tmp = trajectory.current;
		if (trajectory.current == trajectory.head)
		{
			trajectory.head = trajectory.current->next;
		}

		if (trajectory.current == trajectory.tail)
		{
			trajectory.tail = trajectory.current->prev;
		}

		if (trajectory.current->prev)
		{
			trajectory.current->prev->next = trajectory.current->next;
		}

		if (trajectory.current->next)
		{
			trajectory.current->next->prev = trajectory.current->prev;
		}

		if(trajectory.current->prev)
		{
			trajectory.current = trajectory.current->prev;
		}
		else
		{
			trajectory.current = trajectory.current->next;
		}

		sprintf(buffer, "Removed %d", tmp->id);
		sr_ecp_msg->message(buffer);
		delete tmp;

		--trajectory.count;
		--trajectory.position;

		if (!trajectory.position && trajectory.count)
		{
			trajectory.position = 1;
		}

		print_trajectory();

		if (trajectory.current)
		{
			move_to_current();
		}
	}
}

//
//void AndroidTeach::handleB(void)
//{
//	buttonsPressed.buttonB = 0;
//
//	if (has_filenames)
//	{
//		save_trajectory();
//	}
//}
//
//void AndroidTeach::handleA(void)
//{
//	buttonsPressed.buttonA = 0;
//
//	message.led_change = true;
//	message.rumble = false;
//
//	switch (gen)
//	{
//		case 0:
//			message.led_status = 0x1 | 0x8;
//			break;
//		case 1:
//			message.led_status = 0x2 | 0x8;
//			break;
//		case 2:
//			message.led_status = 0x4 | 0x8;
//			break;
//	}
//
//	((ecp_mp::sensor::wiimote*) sensor_m[ecp_mp::sensor::SENSOR_WIIMOTE])->send_reading(message);
//
//	g->Move();
//
//	gg->Move();
//
//	message.led_change = true;
//	message.led_status &= 0x1 | 0x2 | 0x4;
//	message.rumble = false;
//	((ecp_mp::sensor::wiimote*) sensor_m[ecp_mp::sensor::SENSOR_WIIMOTE])->send_reading(message);
//}
//

void AndroidTeach::handlePlus(void)
{
//	buttonsPressed.buttonPlus = 0;

	node* current = new node;
	current->id = ++cnt;

	current->position = gg->get_position_vector();

	if(!current->position.size()) return;

	if (trajectory.current)
	{
		current->next = trajectory.current->next;
		current->prev = trajectory.current;
		trajectory.current->next = current;

		if (trajectory.tail == trajectory.current)
		{
			trajectory.tail = current;
		}
		trajectory.current = current;
	}
	else
	{
		trajectory.current = trajectory.head = trajectory.tail = current;
	}

	++trajectory.position;
	++trajectory.count;

	sprintf(buffer, "Added %s %d: %.4f %.4f %.4f %.4f %.4f %.4f %.4f", type.c_str(), current->id, current->position[0], current->position[1], current->position[2], current->position[3], current->position[4], current->position[5], axis_num > 6 ? current->position[6] : 0);
	sr_ecp_msg->message(buffer);

	print_trajectory();
}

//void AndroidTeach::move_to_first(void)
//{
////	buttonsPressed.down = 0;
//	if (trajectory.count > 0)
//	{
//		do
//		{
//			if(trajectory.current->prev)
//			{
//				trajectory.current = trajectory.current->prev;
//				trajectory.position -= 1;
//			}
//
//			coordinates[0] = trajectory.current->position[0];
//			coordinates[1] = trajectory.current->position[1];
//			coordinates[2] = trajectory.current->position[2];
//			coordinates[3] = trajectory.current->position[3];
//			coordinates[4] = trajectory.current->position[4];
//			coordinates[5] = trajectory.current->position[5];
//
//			sg->load_absolute_angle_axis_trajectory_pose(coordinates);
//		}
//		while(trajectory.current->prev);
//
//		if(sg->calculate_interpolate())
//		{
//			sg->Move();
//		}
//	}
//}

//void AndroidTeach::move_to_last(void)
//{
////	buttonsPressed.up = 0;
//	if (trajectory.count > 0)
//	{
//		sg->reset();
//		sg->set_absolute();
//		do
//		{
//			if(trajectory.current->next)
//			{
//				trajectory.current = trajectory.current->next;
//				trajectory.position += 1;
//			}
//
//			coordinates[0] = trajectory.current->position[0];
//			coordinates[1] = trajectory.current->position[1];
//			coordinates[2] = trajectory.current->position[2];
//			coordinates[3] = trajectory.current->position[3];
//			coordinates[4] = trajectory.current->position[4];
//			coordinates[5] = trajectory.current->position[5];
//
//			sg->load_absolute_angle_axis_trajectory_pose(coordinates);
//		}
//		while(trajectory.current->next);
//
//		if(sg->calculate_interpolate())
//		{
//			sg->Move();
//		}
//	}
//}

void AndroidTeach::move_to_current(void)
{
	if (trajectory.current)
	{
		sprintf(buffer, "Move to %s %d: %.4f %.4f %.4f %.4f %.4f %.4f %.4f", type.c_str(), trajectory.current->id, trajectory.current->position[0], trajectory.current->position[1], trajectory.current->position[2], trajectory.current->position[3], trajectory.current->position[4], trajectory.current->position[5], axis_num > 6 ? trajectory.current->position[6] : 0);
		sr_ecp_msg->message(buffer);
		sg->reset();


		for(int i = 0; i < axis_num; ++i)
		{
			coordinates[i] = trajectory.current->position[i];
		}

		sg->set_absolute();

		if(pose_spec == lib::ECP_JOINT)
		{
			sg->load_absolute_joint_trajectory_pose(coordinates);
		}
		else
		{
			sg->load_absolute_angle_axis_trajectory_pose(coordinates);
		}

		if(sg->calculate_interpolate())
		{
			sg->Move();
		}
	}

	gg->Move();
}


void AndroidTeach::move_to_position(double position[NUMBER_OF_JOINTS])
{
		sprintf(buffer, "Move to %s: %.4f %.4f %.4f %.4f %.4f %.4f %.4f", type.c_str(), position[0], position[1], position[2], position[3], position[4], position[5], axis_num > 6 ? position[6] : 0);
		sr_ecp_msg->message(buffer);
		sg->reset();


		for(int i = 0; i < axis_num; ++i)
		{
			coordinates[i] = position[i];
		}

		sg->set_absolute();

		if(pose_spec == lib::ECP_JOINT)
		{
			sg->load_absolute_joint_trajectory_pose(coordinates);
		}
		else
		{
			sg->load_absolute_angle_axis_trajectory_pose(coordinates);
		}

		if(sg->calculate_interpolate())
		{
			sg->Move();
		}


	gg->Move();
}


void AndroidTeach::main_task_algorithm(void)
{
	cnt = 0;
	gen = 0;
	velocity = "0.1 0.1 0.1 0.1 0.1 0.1 0.1";
	acceleration = "0.1 0.1 0.1 0.1 0.1 0.1 0.1";
	mode = "ABSOLUTE";
//	type = "XYZ_ANGLE_AXIS";
//	pose_spec = lib::ECP_XYZ_ANGLE_AXIS;

	pose_spec = lib::ECP_JOINT;
	type = "JOINT";
	axis_num = 7;

	perror("start main task algorithm");

	ecp_mp::sensor::EcpMpAndroid * androidSensor = dynamic_cast <ecp_mp::sensor::EcpMpAndroid *> (sensor_m[ecp_mp::sensor::SENSOR_ANDROID]);

//	androidSensor->initiate_reading();

//	sg = new common::generator::newsmooth(*this, lib::ECP_JOINT, 7);
	sg = new irp6ot_m::generator::EcpSmoothGAndroid(*this, lib::ECP_JOINT, 7, androidSensor, &androidState);
	sg->set_debug(true);
	jg = new irp6ot_m::generator::EcpGAndroidJoint(*this, androidSensor, &androidState);

	perror("before get_filenames()");
	has_filenames = get_filenames();

	perror("after get_filenames()");




	//sr_ecp_msg->message("ecp: after if(has_filenames)");  //by OL

//	char resp[30];
//	uint8_t response = 0;
//	if(!trajectory.count)
//	{
//		//sr_ecp_msg->message("ecp: in if(!trajectory.count)");  //by OL
//
//		response = choose_option("Pose specification: [1] Angle Axis, [2] Joint", 2);
//		//response = 2;  //by OL
//		//sprintf(resp,"response: %d",response);
//		//sr_ecp_msg->message(resp);
//		//perror(resp);
//	}
//
//	//sr_ecp_msg->message("ecp: after if(!trajectory.count)");  //by OL
//
//
//	switch(response)
//	{
//		case lib::OPTION_TWO:
//			pose_spec = lib::ECP_JOINT;
//			type = "JOINT";
//			axis_num = 7;
//			break;
//		default:
//			pose_spec = lib::ECP_XYZ_ANGLE_AXIS;
//			axis_num = 6;
//			break;
//	}


//	if(!gg)
//	{
//		gg = new common::generator::get_position(*this, pose_spec, axis_num);
//	}

	perror("przed gg");




	perror("super before coord for loop");


	coordinates = std::vector<double>(axis_num);

//	perror("before before coord for loop");
//
//	gg->Move();
//
//	std::vector<double> current_coordinates = gg->get_position_vector();
//
//	perror("before coord for loop");
//
//
//	for (int i = 0; i < axis_num; ++i)
//	{
//		coordinates[i] = current_coordinates[i];
//	}

	perror("after coord for loop");



	g = jg;

	perror("przed connect to android");

	androidSensor->connectToServer();


	if (has_filenames)
	{
		load_trajectory();
		print_trajectory();

		if(trajectory.count)
		{
			move_to_current();
		}
	}

//	androidSensor->sendStart();
//	androidSensor->disconnect();


	while(!androidSensor->sendStart())
	{
		//nothing to write
	}

	sr_ecp_msg->message ("after start from vsp");

	//TODO dodac uzupelnienie konfiguracji
	androidState.updateCurrentPosition(gg->get_position_vector());

	androidSensor->sendConfiguration();

	while (androidState.connected)
	{

		androidSensor->getReadings();
		//perror("pobranie readings");

		switch (androidState.readings.mode)
		{
			case 0:
				//zadawanie predkosci

				if (androidState.readings.value != 0)
				{
					perror("mode 0, zadawanie predkosci");
					try
					{
						g->Move();

//						delete gg;
//						gg = new common::generator::get_position(*this, lib::ECP_JOINT, 7);
						gg->Move();

						std::vector<double> coord;// = std::vector<double>(axis_num);
						coord = gg->get_position_vector();
						sprintf(buffer, "Move to %s: %.4f %.4f %.4f %.4f %.4f %.4f %.4f", type.c_str(), coord[0], coord[1], coord[2], coord[3], coord[4], coord[5], axis_num > 6 ? coord[6] : 0);
						sr_ecp_msg->message(buffer);

					}
					catch (ecp_mp::sensor::android_teach::NetworkException e)//(common::robot::ECP_error e)
					{
						androidSensor->disconnect();
					}

				}
				else
				{
					androidState.setLimit(0);
				}
			break;
			case 1:
				//zadawanie pozycji
				perror("mode 1, zadawanie pozycji");
				try
				{
					move_to_position(androidState.readings.newJointValue);
					androidSensor->sendEndOfMotion();
				}
				catch (ecp_mp::sensor::android_teach::NetworkException e)//(common::robot::ECP_error e)
				{
					perror("error in movement to position");
				}


				androidState.readings.mode = 0;
			break;
			case 2:
				//kontrola punktow trajektorii

				switch (androidState.readings.value)
				{
					case 1:
						//dodanie wezla
						perror("mode 2, dodanie wezla");
						handlePlus();

					break;

					case 2:
						//usuniecie wezla
						perror("mode 2, usuniecie wezla");
						handleMinus();
					break;
					case 3:
						//przejscie do nastepnego wezla
						perror("mode 2, nastepny wezel");
						move_to_next();
					break;
					case 4:
						//przejscie do porzedniego wezla
						perror("mode 2, poprzedni wezel");
						move_to_prev();
					break;

				}
				perror("mode 2, wykonano");
				androidSensor->sendEndOfMotion();
				perror("mode 2, wyslano zakonczenie ruchu");
			break;
			case 3:
				//nothing to write, after disconnect
			break;
			default:
				sr_ecp_msg->message ("Reply mode from VSP in main loop not ok");
				androidSensor->disconnect();
			break;

		}

	}


//	while (1)
//	{
//
//
//		sensor_m[ecp_mp::sensor::SENSOR_ANDROID]->get_reading();
//		updateButtonsPressed();
//		lastButtons = wii->image;
//
//		if (buttonsPressed.button1 || buttonsPressed.button2)
//		{
//			handle12();
//		}
//		else if (buttonsPressed.buttonA)
//		{
//			handleA();
//		}
//		else if (buttonsPressed.left)
//		{
//			move_to_prev();
//		}
//		else if (buttonsPressed.right)
//		{
//			move_to_next();
//		}
//		else if (buttonsPressed.up)
//		{
//			move_to_last();
//		}
//		else if (buttonsPressed.down)
//		{
//			move_to_first();
//		}
//		else if (buttonsPressed.buttonPlus)
//		{
//			handlePlus();
//		}
//		else if (buttonsPressed.buttonMinus)
//		{
//			handleMinus();
//		}
//		else if (buttonsPressed.buttonHome)
//		{
//			handleHome();
//		}
//		else if (buttonsPressed.buttonB)
//		{
//			handleB();
//		}
//	}
//	move_to_current();


	save_trajectory();

//	ecp_termination_notice();
}





}
} // namespace irp6ot

namespace common {
namespace task {

task_base* return_created_ecp_task(lib::configurator &_config)
{
	return new irp6ot_m::task::AndroidTeach(_config);
}


}
} // namespace common
} // namespace ecp
} // namespace mrrocpp
