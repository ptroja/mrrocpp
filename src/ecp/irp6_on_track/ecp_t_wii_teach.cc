#include <string.h>

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"
#include <fstream>

#include "lib/srlib.h"
#include "ecp_mp/ecp_mp_s_wiimote.h"

#include "ecp/irp6_on_track/ecp_r_irp6ot.h"
#include "ecp/irp6_on_track/ecp_t_wii_teach.h"
#include "lib/mathtr.h"
#include "ecp_t_wii_teach.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot {
namespace task {


wii_teach::wii_teach(lib::configurator &_config) : task(_config)
{
    ecp_m_robot = new robot (*this);
    trajectory.count = trajectory.position = 0;
    trajectory.head = trajectory.tail = trajectory.current = NULL;

    //create Wii-mote virtual sensor object
    sensor_m[lib::SENSOR_WIIMOTE] = new ecp_mp::sensor::wiimote(lib::SENSOR_WIIMOTE, "[vsp_wiimote]", *this, sizeof(lib::sensor_image_t::sensor_union_t::wiimote_t));
    //configure the sensor
    sensor_m[lib::SENSOR_WIIMOTE]->configure_sensor();
}

bool wii_teach::get_file_name(void)
{
    lib::ECP_message ecp_to_ui_msg; // Przesylka z ECP do UI
    lib::UI_reply ui_to_ecp_rep; // Odpowiedz UI do ECP
    uint64_t e; // Kod bledu systemowego

    ecp_to_ui_msg.ecp_message = lib::SAVE_FILE; // Polecenie wprowadzenia nazwy pliku
    strcpy(ecp_to_ui_msg.string, "*.trj"); // Wzorzec nazwy pliku
    // if ( Send (UI_pid, &ecp_to_ui_msg, &ui_to_ecp_rep, sizeof(lib::ECP_message), sizeof(lib::UI_reply)) == -1) {
#if !defined(USE_MESSIP_SRR)
    ecp_to_ui_msg.hdr.type=0;
    if (MsgSend(this->UI_fd, &ecp_to_ui_msg, sizeof(lib::ECP_message), &ui_to_ecp_rep, sizeof(lib::UI_reply)) < 0)
#else
    int status;
    if(messip_send(this->UI_fd, 0, 0, &ecp_to_ui_msg, sizeof(lib::ECP_message),
    		&status, &ui_to_ecp_rep, sizeof(lib::UI_reply), MESSIP_NOTIMEOUT) < 0)
#endif
    {// by Y&W
        e = errno;
        perror("ECP: Send() to UI failed");
        sr_ecp_msg->message(lib::SYSTEM_ERROR, e, "ECP: Send() to UI failed");
        throw common::ecp_robot::ECP_error(lib::SYSTEM_ERROR, (uint64_t) 0);
    }
    
    if (ui_to_ecp_rep.reply == lib::QUIT)
    { // Nie wybrano nazwy pliku lub zrezygnowano z zapisu
        return false;
    }

    strncpy(path,ui_to_ecp_rep.path,79);
    strncpy(filename,ui_to_ecp_rep.filename,19);
    return true;
}

void wii_teach::save_trajectory(void)
{
      char buffer[80];
      uint64_t e; // Kod bledu systemowego

      if (chdir(path) != 0)
      {
        perror(path);
        throw common::ecp_robot::ECP_error(lib::NON_FATAL_ERROR, NON_EXISTENT_DIRECTORY);
      }

      std::ofstream to_file(filename); // otworz plik do zapisu
      e = errno;
      if (!to_file)
      {
        perror(filename);
        throw common::ecp_robot::ECP_error(lib::NON_FATAL_ERROR, NON_EXISTENT_FILE);
      }
      else
      {
          node* current = trajectory.head;
          to_file << "XYZ_ANGLE_AXIS" << '\n';
          to_file << trajectory.count << '\n';

          while(current)
          {
              to_file << current->position[0] << ' ';
              to_file << current->position[1] << ' ';
              to_file << current->position[2] << ' ';
              to_file << current->position[3] << ' ';
              to_file << current->position[4] << ' ';
              to_file << current->position[5] << ' ';
              to_file << '\n';

              current = current->next;
          }
      }

      sprintf(buffer,"Trajectory saved to %s/%s",path,filename);
      sr_ecp_msg->message(buffer);
}

void wii_teach::updateButtonsPressed(void)
{
    buttonsPressed.left = !lastButtons.left && sensor_m[lib::SENSOR_WIIMOTE]->image.sensor_union.wiimote.left;
    buttonsPressed.right = !lastButtons.right && sensor_m[lib::SENSOR_WIIMOTE]->image.sensor_union.wiimote.right;
    buttonsPressed.up = !lastButtons.up && sensor_m[lib::SENSOR_WIIMOTE]->image.sensor_union.wiimote.up;
    buttonsPressed.down = !lastButtons.down && sensor_m[lib::SENSOR_WIIMOTE]->image.sensor_union.wiimote.down;
    buttonsPressed.buttonA = !lastButtons.buttonA && sensor_m[lib::SENSOR_WIIMOTE]->image.sensor_union.wiimote.buttonA;
    buttonsPressed.buttonB = !lastButtons.buttonB && sensor_m[lib::SENSOR_WIIMOTE]->image.sensor_union.wiimote.buttonB;
    buttonsPressed.button1 = !lastButtons.button1 && sensor_m[lib::SENSOR_WIIMOTE]->image.sensor_union.wiimote.button1;
    buttonsPressed.button2 = !lastButtons.button2 && sensor_m[lib::SENSOR_WIIMOTE]->image.sensor_union.wiimote.button2;
    buttonsPressed.buttonPlus = !lastButtons.buttonPlus && sensor_m[lib::SENSOR_WIIMOTE]->image.sensor_union.wiimote.buttonPlus;
    buttonsPressed.buttonMinus = !lastButtons.buttonMinus && sensor_m[lib::SENSOR_WIIMOTE]->image.sensor_union.wiimote.buttonMinus;
    buttonsPressed.buttonHome = !lastButtons.buttonHome && sensor_m[lib::SENSOR_WIIMOTE]->image.sensor_union.wiimote.buttonHome;
}

void wii_teach::print_trajectory(void)
{
    char buffer[80];
    node* current = trajectory.head;
    int i = 0;
    
    sr_ecp_msg->message("=== Trajektoria ===");
    while(current)
    {
        sprintf(buffer,"Pozycja %d: %.4f %.4f %.4f %.4f %.4f %.4f",++i,current->position[0],current->position[1],current->position[2],current->position[3],current->position[4],current->position[5]);
        sr_ecp_msg->message(buffer);
        current = current->next;
    }
    sr_ecp_msg->message("=== Trajektoria - koniec ===");
}

void wii_teach::move_to_current(void)
{
    char buffer[80];
    if(trajectory.current)
    {
        sprintf(buffer,"Move to %d: %.4f %.4f %.4f %.4f %.4f %.4f",trajectory.current->id,trajectory.current->position[0],trajectory.current->position[1],trajectory.current->position[2],trajectory.current->position[3],trajectory.current->position[4],trajectory.current->position[5]);
        sr_ecp_msg->message(buffer);
        sg->set_absolute();
        sg->load_coordinates(lib::XYZ_ANGLE_AXIS,trajectory.current->position[0],trajectory.current->position[1],trajectory.current->position[2],trajectory.current->position[3],trajectory.current->position[4],trajectory.current->position[5],0.074,0,true);
        sg->Move();
    }
}

void wii_teach::main_task_algorithm(void)
{
    int cnt = 0;
    char buffer[80];
    struct lib::ECP_VSP_MSG message;

    sg = new common::generator::smooth2(*this,true);
    wg = new irp6ot::generator::wii_teach(*this,sensor_m[lib::SENSOR_WIIMOTE],sg);

    if(get_file_name())
    {
        while(1)
        {
            sensor_m[lib::SENSOR_WIIMOTE]->get_reading();
            updateButtonsPressed();
            lastButtons = sensor_m[lib::SENSOR_WIIMOTE]->image.sensor_union.wiimote;

            if(buttonsPressed.buttonB)
            {
                buttonsPressed.buttonB = 0;
                message.i_code = lib::VSP_CONFIGURE_SENSOR;
                message.wii_command.led_status = 0xF;
                ((ecp_mp::sensor::wiimote*)sensor_m[lib::SENSOR_WIIMOTE])->send_reading(message);
                wg->Move();
                message.wii_command.led_status = 0x0;
                ((ecp_mp::sensor::wiimote*)sensor_m[lib::SENSOR_WIIMOTE])->send_reading(message);
                buttonsPressed.buttonB = 0;
            }
            else
            {
                if(buttonsPressed.left)
                {
                    buttonsPressed.left = 0;
                    if(trajectory.position > 1)
                    {
                        --trajectory.position;
                        trajectory.current = trajectory.current->prev;
                        move_to_current();
                    }
                }
                else if(buttonsPressed.right)
                {
                    buttonsPressed.right = 0;
                    if(trajectory.position < trajectory.count)
                    {
                        ++trajectory.position;
                        trajectory.current = trajectory.current->next;
                        move_to_current();
                    }
                }
                else if(buttonsPressed.up)
                {
                    buttonsPressed.up = 0;
                    if(trajectory.count > 0)
                    {
                        trajectory.position = trajectory.count;
                        trajectory.current = trajectory.tail;
                        move_to_current();
                    }
                }
                else if(buttonsPressed.down)
                {
                    buttonsPressed.down = 0;
                    if(trajectory.count > 0)
                    {
                        trajectory.position = 1;
                        trajectory.current = trajectory.head;
                        move_to_current();
                    }
                }
                else if(buttonsPressed.buttonPlus)
                {
                    buttonsPressed.buttonPlus = 0;

                    node* current = new node;
                    current->id = ++cnt;
                    current->position[0] = ecp_m_robot->EDP_data.current_XYZ_AA_arm_coordinates[0];
                    current->position[1] = ecp_m_robot->EDP_data.current_XYZ_AA_arm_coordinates[1];
                    current->position[2] = ecp_m_robot->EDP_data.current_XYZ_AA_arm_coordinates[2];
                    current->position[3] = ecp_m_robot->EDP_data.current_XYZ_AA_arm_coordinates[3];
                    current->position[4] = ecp_m_robot->EDP_data.current_XYZ_AA_arm_coordinates[4];
                    current->position[5] = ecp_m_robot->EDP_data.current_XYZ_AA_arm_coordinates[5];

                    if(trajectory.current)
                    {
                        current->next = trajectory.current->next;
                        current->prev = trajectory.current;
                        trajectory.current->next = current;
                        if(trajectory.tail == trajectory.current) trajectory.tail = current;
                        trajectory.current = current;
                    }
                    else
                    {
                        trajectory.current = trajectory.head = trajectory.tail = current;
                    }

                    ++trajectory.position;
                    ++trajectory.count;

                    sprintf(buffer,"Added %d: %.4f %.4f %.4f %.4f %.4f %.4f",trajectory.current->id,trajectory.current->position[0],trajectory.current->position[1],trajectory.current->position[2],trajectory.current->position[3],trajectory.current->position[4],trajectory.current->position[5]);
                    sr_ecp_msg->message(buffer);

                    print_trajectory();
                }
                else if(buttonsPressed.buttonMinus)
                {
                    buttonsPressed.buttonMinus = 0;
                    if(trajectory.position > 0)
                    {
                        node* tmp = trajectory.current;
                        if(trajectory.current == trajectory.head)
                        {
                            trajectory.head = trajectory.current->next;
                        }
                        if(trajectory.current == trajectory.tail)
                        {
                            trajectory.tail = trajectory.current->prev;
                        }
                        if(trajectory.current->prev)
                        {
                            trajectory.current->prev->next = trajectory.current->next;
                        }
                        if(trajectory.current->next)
                        {
                            trajectory.current->next->prev = trajectory.current->prev;
                        }
                        trajectory.current = trajectory.current->prev;

                        sprintf(buffer,"Removed %d",tmp->id);
                        sr_ecp_msg->message(buffer);
                        delete tmp;

                        --trajectory.count;
                        --trajectory.position;
                        if(!trajectory.position && trajectory.count) trajectory.position = 1;
                        print_trajectory();

                        if(trajectory.current) move_to_current();
                    }
                }
                else if(buttonsPressed.buttonHome)
                {
                    buttonsPressed.buttonHome = 0;
                    if(trajectory.position > 0)
                    {
                        int old = trajectory.current->id;
                        trajectory.current->id = ++cnt;
                        trajectory.current->position[0] = ecp_m_robot->EDP_data.current_XYZ_AA_arm_coordinates[0];
                        trajectory.current->position[1] = ecp_m_robot->EDP_data.current_XYZ_AA_arm_coordinates[1];
                        trajectory.current->position[2] = ecp_m_robot->EDP_data.current_XYZ_AA_arm_coordinates[2];
                        trajectory.current->position[3] = ecp_m_robot->EDP_data.current_XYZ_AA_arm_coordinates[3];
                        trajectory.current->position[4] = ecp_m_robot->EDP_data.current_XYZ_AA_arm_coordinates[4];
                        trajectory.current->position[5] = ecp_m_robot->EDP_data.current_XYZ_AA_arm_coordinates[5];

                        sprintf(buffer,"Changed %d: %.4f %.4f %.4f %.4f %.4f %.4f",trajectory.current->id,trajectory.current->position[0],trajectory.current->position[1],trajectory.current->position[2],trajectory.current->position[3],trajectory.current->position[4],trajectory.current->position[5]);
                        sr_ecp_msg->message(buffer);
                    }

                    print_trajectory();
                }
                else if(buttonsPressed.buttonA)
                {
                    buttonsPressed.buttonA = 0;
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

task* return_created_ecp_task (lib::configurator &_config)
{
	return new irp6ot::task::wii_teach(_config);
}

}
} // namespace common
} // namespace ecp
} // namespace mrrocpp


