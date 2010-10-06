#include <cstring>

#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"
#include <fstream>

#include "base/lib/sr/srlib.h"
#include "application/wii_teach/sensor/ecp_mp_s_wiimote.h"

#include "base/ecp/ecp_task.h"
#include "robot/irp6ot_m/ecp_r_irp6ot_m.h"
#include "application/wii_teach/ecp_t_wii_teach.h"
#include "base/lib/mrmath/mrmath.h"
#include "ecp_t_wii_teach.h"

#if defined(USE_MESSIP_SRR)
#include "base/lib/messip/messip_dataport.h"
#endif

namespace mrrocpp {
namespace ecp {
namespace irp6ot_m {
namespace task {

wii_teach::wii_teach(lib::configurator &_config) :
	task(_config)
{
	ecp_m_robot = new robot(*this);
	trajectory.count = trajectory.position = 0;
	trajectory.head = trajectory.tail = trajectory.current = NULL;

	//create Wii-mote virtual sensor object
	sensor_m[ecp_mp::sensor::SENSOR_WIIMOTE]
			= new ecp_mp::sensor::wiimote(ecp_mp::sensor::SENSOR_WIIMOTE, "[vsp_wiimote]", *this->sr_ecp_msg, this->config);
	//configure the sensor
	sensor_m[ecp_mp::sensor::SENSOR_WIIMOTE]->configure_sensor();
	coordinates = std::vector<double>(6);
}

int wii_teach::load_trajectory()
{
	char buffer[200];
	uint64_t e; // Kod bledu systemowego

	if (chdir(path) != 0) {
		perror(path);
		throw common::robot::ECP_error(lib::NON_FATAL_ERROR, NON_EXISTENT_DIRECTORY);
	}

	std::ifstream from_file(filename); // otworz plik do zapisu
	e = errno;
	if (!from_file) {
		perror(filename);
		throw common::robot::ECP_error(lib::NON_FATAL_ERROR, NON_EXISTENT_FILE);
	}

	if (chdir(gripper_path) != 0) {
		perror(gripper_path);
		throw common::robot::ECP_error(lib::NON_FATAL_ERROR, NON_EXISTENT_DIRECTORY);
	}

	std::ifstream from_file_gripper(gripper_filename); // otworz plik do zapisu
	e = errno;
	if (!from_file_gripper) {
		perror(gripper_filename);
		throw common::robot::ECP_error(lib::NON_FATAL_ERROR, NON_EXISTENT_FILE);
	}

	node* current = NULL;
	trajectory.position = 0;
	trajectory.count = 0;
	std::string type;
	int count;

	from_file >> type;
	from_file >> count;
	from_file_gripper >> type;
	from_file_gripper >> count;
	while (!from_file.eof() && !from_file_gripper.eof()) {
		if (current) {
			current->next = new node();
			current->next->prev = current;
		} else {
			current = new node();
		}

		if (!trajectory.head) {
			trajectory.head = current;
		}
		trajectory.tail = current;

		from_file >> current->position[0];
		from_file >> current->position[1];
		from_file >> current->position[2];
		from_file >> current->position[3];
		from_file >> current->position[4];
		from_file >> current->position[5];

		from_file_gripper >> current->gripper;

		trajectory.position = 1;
		++trajectory.count;

		sprintf(buffer, "Loaded %d: %.4f %.4f %.4f %.4f %.4f %.4f %.4f", trajectory.count, current->position[0], current->position[1], current->position[2], current->position[3], current->position[4], current->position[5], current->gripper);
		sr_ecp_msg->message(buffer);
	}

	trajectory.current = trajectory.head;

	return 0;
}

bool wii_teach::get_filenames(void)
{
	lib::ECP_message ecp_to_ui_msg; // Przesylka z ECP do UI
	lib::UI_reply ui_to_ecp_rep; // Odpowiedz UI do ECP
	uint64_t e; // Kod bledu systemowego

	ecp_to_ui_msg.ecp_message = lib::SAVE_FILE; // Polecenie wprowadzenia nazwy pliku
	strcpy(ecp_to_ui_msg.string, "*.trj"); // Wzorzec nazwy pliku
	// if ( Send (UI_pid, &ecp_to_ui_msg, &ui_to_ecp_rep, sizeof(lib::ECP_message), sizeof(lib::UI_reply)) == -1) {
#if !defined(USE_MESSIP_SRR)
	ecp_to_ui_msg.hdr.type = 0;
	if (MsgSend(this->UI_fd, &ecp_to_ui_msg, sizeof(lib::ECP_message), &ui_to_ecp_rep, sizeof(lib::UI_reply)) < 0)
#else
	if(messip::port_send(this->UI_fd, 0, 0, ecp_to_ui_msg, ui_to_ecp_rep) < 0)
#endif
	{// by Y&W
		e = errno;
		perror("ecp: Send() to UI failed");
		sr_ecp_msg->message(lib::SYSTEM_ERROR, e, "ecp: Send() to UI failed");
		throw common::robot::ECP_error(lib::SYSTEM_ERROR, 0);
	}

	if (ui_to_ecp_rep.reply == lib::QUIT) { // Nie wybrano nazwy pliku lub zrezygnowano z zapisu
		return false;
	}

	strncpy(path, ui_to_ecp_rep.path, 79);
	strncpy(filename, ui_to_ecp_rep.filename, 19);

	ecp_to_ui_msg.ecp_message = lib::SAVE_FILE; // Polecenie wprowadzenia nazwy pliku
	strcpy(ecp_to_ui_msg.string, "*.trj"); // Wzorzec nazwy pliku
	// if ( Send (UI_pid, &ecp_to_ui_msg, &ui_to_ecp_rep, sizeof(lib::ECP_message), sizeof(lib::UI_reply)) == -1) {
#if !defined(USE_MESSIP_SRR)
	ecp_to_ui_msg.hdr.type = 0;
	if (MsgSend(this->UI_fd, &ecp_to_ui_msg, sizeof(lib::ECP_message), &ui_to_ecp_rep, sizeof(lib::UI_reply)) < 0)
#else
	if(messip::port_send(this->UI_fd, 0, 0, ecp_to_ui_msg, ui_to_ecp_rep) < 0)
#endif
	{// by Y&W
		e = errno;
		perror("ecp: Send() to UI failed");
		sr_ecp_msg->message(lib::SYSTEM_ERROR, e, "ecp: Send() to UI failed");
		throw common::robot::ECP_error(lib::SYSTEM_ERROR, 0);
	}

	if (ui_to_ecp_rep.reply == lib::QUIT) { // Nie wybrano nazwy pliku lub zrezygnowano z zapisu
		return false;
	}

	strncpy(gripper_path, ui_to_ecp_rep.path, 79);
	strncpy(gripper_filename, ui_to_ecp_rep.filename, 19);
	return true;
}

void wii_teach::save_trajectory(void)
{
	char buffer[200];
	uint64_t e; // Kod bledu systemowego

	if (chdir(path) != 0) {
		perror(path);
		throw common::robot::ECP_error(lib::NON_FATAL_ERROR, NON_EXISTENT_DIRECTORY);
	}

	std::ofstream to_file(filename); // otworz plik do zapisu
	e = errno;
	if (!to_file) {
		perror(filename);
		throw common::robot::ECP_error(lib::NON_FATAL_ERROR, NON_EXISTENT_FILE);
	}
	std::ofstream to_file_gripper(gripper_filename); // otworz plik do zapisu
	e = errno;
	if (!to_file_gripper) {
		perror(gripper_filename);
		throw common::robot::ECP_error(lib::NON_FATAL_ERROR, NON_EXISTENT_FILE);
	}

	node* current = trajectory.head;
	to_file << "XYZ_ANGLE_AXIS" << '\n';
	to_file << trajectory.count << '\n';

	while (current) {
		to_file << current->position[0] << ' ';
		to_file << current->position[1] << ' ';
		to_file << current->position[2] << ' ';
		to_file << current->position[3] << ' ';
		to_file << current->position[4] << ' ';
		to_file << current->position[5] << ' ';
		to_file_gripper << current->gripper;

		to_file << '\n';
		to_file_gripper << '\n';

		current = current->next;
	}

	sprintf(buffer, "Trajectory saved to %s/%s", path, filename);
	sr_ecp_msg->message(buffer);
}

void wii_teach::updateButtonsPressed(void)
{
	ecp_mp::sensor::wiimote * wii = dynamic_cast <ecp_mp::sensor::wiimote *> (sensor_m[ecp_mp::sensor::SENSOR_WIIMOTE]);

	buttonsPressed.left = !lastButtons.left && wii->image.left;
	buttonsPressed.right = !lastButtons.right && wii->image.right;
	buttonsPressed.up = !lastButtons.up && wii->image.up;
	buttonsPressed.down = !lastButtons.down && wii->image.down;
	buttonsPressed.buttonA = !lastButtons.buttonA && wii->image.buttonA;
	buttonsPressed.buttonB = !lastButtons.buttonB && wii->image.buttonB;
	buttonsPressed.button1 = !lastButtons.button1 && wii->image.button1;
	buttonsPressed.button2 = !lastButtons.button2 && wii->image.button2;
	buttonsPressed.buttonPlus = !lastButtons.buttonPlus && wii->image.buttonPlus;
	buttonsPressed.buttonMinus = !lastButtons.buttonMinus && wii->image.buttonMinus;
	buttonsPressed.buttonHome = !lastButtons.buttonHome && wii->image.buttonHome;
}

void wii_teach::print_trajectory(void)
{
	char buffer[200];
	node* current = trajectory.head;
	int i = 0;

	sr_ecp_msg->message("=== Trajektoria ===");
	while (current) {
		sprintf(buffer, "Pozycja %d: %.4f %.4f %.4f %.4f %.4f %.4f %.4f", ++i, current->position[0], current->position[1], current->position[2], current->position[3], current->position[4], current->position[5], current->gripper);
		sr_ecp_msg->message(buffer);
		current = current->next;
	}
	sr_ecp_msg->message("=== Trajektoria - koniec ===");
}

void wii_teach::move_to_current(void)
{

	char buffer[200];
	if (trajectory.current) {
		sprintf(buffer, "Move to %d: %.4f %.4f %.4f %.4f %.4f %.4f %.4f", trajectory.current->id, trajectory.current->position[0], trajectory.current->position[1], trajectory.current->position[2], trajectory.current->position[3], trajectory.current->position[4], trajectory.current->position[5], trajectory.current->gripper);
		sr_ecp_msg->message(buffer);
		sg->reset();
		coordinates[0] = -0.150;
		coordinates[1] = 1.12;
		coordinates[2] = -0.1;
		coordinates[3] = 0.729*3.14;
		coordinates[4] = 0.685*3.14;
		coordinates[5] = -0.001*3.14;
		sg->set_absolute();
		sg->load_absolute_angle_axis_trajectory_pose(coordinates);
		sg->Move();
	}

}

void wii_teach::main_task_algorithm(void)
{
	int cnt = 0;
	char buffer[200];
	int gen = 0;

	ecp_mp::sensor::wiimote * wii = dynamic_cast <ecp_mp::sensor::wiimote *> (sensor_m[ecp_mp::sensor::SENSOR_WIIMOTE]);

	sg = new common::generator::newsmooth(*this, lib::ECP_XYZ_ANGLE_AXIS, 6);
	ag = new irp6ot_m::generator::wii_absolute(*this, wii);
	rg = new irp6ot_m::generator::wii_relative(*this, wii);
	jg = new irp6ot_m::generator::wii_joint(*this, wii);

	bool has_filenames = false;//get_filenames();
	if (has_filenames) {
		load_trajectory();
		move_to_current();
	}

	common::generator::generator* g = ag;
	//	int i = 0;

	ecp_mp::sensor::wii_command_t message;

	message.led_change = true;
	message.rumble = false;
	switch (gen)
	{
		case 0:
			message.led_status = 0x1;
			break;
		case 1:
			message.led_status = 0x2;
			break;
		case 2:
			message.led_status = 0x4;
			break;
	}
	wii->send_reading(message);

	while (1) {
		sensor_m[ecp_mp::sensor::SENSOR_WIIMOTE]->get_reading();
		updateButtonsPressed();
		lastButtons = wii->image;

		if (buttonsPressed.button1 || buttonsPressed.button2) {
			if (buttonsPressed.button1) {
				buttonsPressed.button1 = 0;
				gen = ++gen % 3;
			} else if (buttonsPressed.button2) {
				buttonsPressed.button2 = 0;
				gen = --gen % 3;
				if (gen < 0)
					gen = 2;
			}
			message.led_change = true;
			message.rumble = false;
			switch (gen)
			{
				case 0:
					g = ag;
					message.led_status = 0x1;
					break;
				case 1:
					g = rg;
					message.led_status = 0x2;
					break;
				case 2:
					g = jg;
					message.led_status = 0x4;
					break;
			}
			((ecp_mp::sensor::wiimote*) sensor_m[ecp_mp::sensor::SENSOR_WIIMOTE])->send_reading(message);
		} else if (buttonsPressed.buttonA) {
			buttonsPressed.buttonA = 0;

			message.led_change = true;
			message.rumble = false;

			switch (gen)
			{
				case 0:
					message.led_status = 0x1 | 0x8;
					break;
				case 1:
					message.led_status = 0x2 | 0x8;
					break;
				case 2:
					message.led_status = 0x4 | 0x8;
					break;
			}
			((ecp_mp::sensor::wiimote*) sensor_m[ecp_mp::sensor::SENSOR_WIIMOTE])->send_reading(message);

			g->Move();

			message.led_change = true;
			message.led_status &= 0x1 | 0x2 | 0x4;
			message.rumble = false;
			((ecp_mp::sensor::wiimote*) sensor_m[ecp_mp::sensor::SENSOR_WIIMOTE])->send_reading(message);

		} else {
			if (buttonsPressed.left) {
				buttonsPressed.left = 0;
				if (trajectory.position > 1) {
					--trajectory.position;
					trajectory.current = trajectory.current->prev;
					move_to_current();
				}
			} else if (buttonsPressed.right) {
				buttonsPressed.right = 0;
				if (trajectory.position < trajectory.count) {
					++trajectory.position;
					trajectory.current = trajectory.current->next;
					move_to_current();
				}
			} else if (buttonsPressed.up) {
				buttonsPressed.up = 0;
				if (trajectory.count > 0) {
					trajectory.position = trajectory.count;
					trajectory.current = trajectory.tail;
					move_to_current();
				}
			} else if (buttonsPressed.down) {
				buttonsPressed.down = 0;
				if (trajectory.count > 0) {
					trajectory.position = 1;
					trajectory.current = trajectory.head;
					move_to_current();
				}
			} else if (buttonsPressed.buttonPlus) {
				buttonsPressed.buttonPlus = 0;

				node* current = new node;
				current->id = ++cnt;

				homog_matrix.set_from_frame_tab(ecp_m_robot->reply_package.arm.pf_def.arm_frame);
				lib::Xyz_Angle_Axis_vector tmp_vector;
				homog_matrix.get_xyz_angle_axis(tmp_vector);
				tmp_vector.to_table(current->position);

				if (trajectory.current) {
					current->next = trajectory.current->next;
					current->prev = trajectory.current;
					trajectory.current->next = current;
					if (trajectory.tail == trajectory.current)
						trajectory.tail = current;
					trajectory.current = current;
				} else {
					trajectory.current = trajectory.head = trajectory.tail = current;
				}

				++trajectory.position;
				++trajectory.count;

				sprintf(buffer, "Added %d: %.4f %.4f %.4f %.4f %.4f %.4f %.4f", current->id, current->position[0], current->position[1], current->position[2], current->position[3], current->position[4], current->position[5], current->gripper);
				sr_ecp_msg->message(buffer);

				print_trajectory();
			} else if (buttonsPressed.buttonMinus) {
				buttonsPressed.buttonMinus = 0;
				if (trajectory.position > 0) {
					node* tmp = trajectory.current;
					if (trajectory.current == trajectory.head) {
						trajectory.head = trajectory.current->next;
					}
					if (trajectory.current == trajectory.tail) {
						trajectory.tail = trajectory.current->prev;
					}
					if (trajectory.current->prev) {
						trajectory.current->prev->next = trajectory.current->next;
					}
					if (trajectory.current->next) {
						trajectory.current->next->prev = trajectory.current->prev;
					}
					trajectory.current = trajectory.current->prev;

					sprintf(buffer, "Removed %d", tmp->id);
					sr_ecp_msg->message(buffer);
					delete tmp;

					--trajectory.count;
					--trajectory.position;
					if (!trajectory.position && trajectory.count)
						trajectory.position = 1;
					print_trajectory();

					if (trajectory.current)
						move_to_current();
				}
			} else if (buttonsPressed.buttonHome) {
				buttonsPressed.buttonHome = 0;
				if (trajectory.position > 0) {
					//			int old = trajectory.current->id;
					trajectory.current->id = ++cnt;

					homog_matrix.set_from_frame_tab(ecp_m_robot->reply_package.arm.pf_def.arm_frame);
					lib::Xyz_Angle_Axis_vector tmp_vector;
					homog_matrix.get_xyz_angle_axis(tmp_vector);
					tmp_vector.to_table(trajectory.current->position);

					sprintf(buffer, "Changed %d: %.4f %.4f %.4f %.4f %.4f %.4f %.4f", trajectory.current->id, trajectory.current->position[0], trajectory.current->position[1], trajectory.current->position[2], trajectory.current->position[3], trajectory.current->position[4], trajectory.current->position[5], trajectory.current->gripper);
					sr_ecp_msg->message(buffer);
				}

				print_trajectory();
			} else if (buttonsPressed.buttonB) {
				buttonsPressed.buttonB = 0;
				if (has_filenames) {
					save_trajectory();
				}
			}
		}
	}

	ecp_termination_notice();
}

}
} // namespace irp6ot

namespace common {
namespace task {

task* return_created_ecp_task(lib::configurator &_config)
{
	return new irp6ot_m::task::wii_teach(_config);
}

}
} // namespace common
} // namespace ecp
} // namespace mrrocpp


