//
// READER - watek do buforowania danych pomiarowych i ich zapisu do pliku
// Date: maj 2006
//

#include <stdio.h>
#include <ctype.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <math.h>
#include <iostream>
#include <fstream>
#include <signal.h>
#include <errno.h>
#include <sys/wait.h>
#include <sys/types.h>
#include <sys/neutrino.h>
#include <sys/sched.h>
#include <sys/iofunc.h>
#include <sys/dispatch.h>
#include <errno.h>
#include <pthread.h>
#include <sys/netmgr.h>
#include <time.h>

#include "common/typedefs.h"
#include "common/impconst.h"
#include "common/com_buf.h"
#include "lib/mis_fun.h"
#include "edp/common/edp.h"

reader_buffer rb_obj;

extern edp_effector *master;  // by Y

inline void check_config(const char* string, uint8_t* input )
{
    if ((master->config.exists(string))&&(master->config.return_int_value(string)))
        *input=1;
    else
        *input=0;
}

void * edp_irp6s_and_conv_effector::reader_thread(void* arg)
{
    int i;
    uint64_t k;
    uint64_t nr_of_samples;  // maksymalna liczba pomiarow
    reader_data* r_measptr; // tablica - bufor cykliczny z danymi pomiarowymi
    uint64_t msr_nr; // numer pomiaru
    int przepelniony; // czy bufor byl przepelniony
    int msr_counter; // liczba pomiarow, ktore maja byc zapisane do pliku
    name_attach_t *my_attach;	// nazwa kanalu komunikacyjnego
    uint64_t e;     // kod bledu systemowego
    _pulse_msg ui_msg;// wiadomosc z ui
    int wyjscie;
    int msg_return;

    bool ui_trigger = false; // specjalny puls z UI

    int file_counter=0;
    time_t time_of_day;
    char file_name[50];
    char file_date[40];
    char config_file_with_dir[80];

    struct sigevent stop_event; // do oblugi pulsu stopu

    // czytanie konfiguracji
    char*	reader_meassures_dir;
    if (master->config.exists("reader_meassures_dir"))
    {
        reader_meassures_dir = master->config.return_string_value("reader_meassures_dir", "[ui]");
    }
    else
    {
        reader_meassures_dir = master->config.return_default_reader_measures_path();
    }

    char* robot_name = master->config.return_string_value("reader_attach_point");

    if (master->config.exists("reader_samples"))
        nr_of_samples=master->config.return_int_value("reader_samples");
    else
        nr_of_samples=1000;

    rb_obj.reader_cnf.step=1;
    check_config("servo_tryb", &(rb_obj.reader_cnf.servo_tryb));
    check_config("msec", &(rb_obj.reader_cnf.msec));

    char tmp_string[50];
    char tmp2_string[3];

    for (int j=0; j<MAX_SERVOS_NR; j++)
    {
        sprintf(tmp2_string, "%d", j);

        strcpy(tmp_string,"desired_inc_");
        strcat(tmp_string, tmp2_string);
        check_config(tmp_string, &(rb_obj.reader_cnf.desired_inc[j]));

        strcpy(tmp_string,"current_inc_");
        strcat(tmp_string, tmp2_string);
        check_config(tmp_string, &(rb_obj.reader_cnf.current_inc[j]));

        strcpy(tmp_string,"pwm_");
        strcat(tmp_string, tmp2_string);
        check_config(tmp_string, &(rb_obj.reader_cnf.pwm[j]));

        strcpy(tmp_string,"uchyb_");
        strcat(tmp_string, tmp2_string);
        check_config(tmp_string, &(rb_obj.reader_cnf.uchyb[j]));

        strcpy(tmp_string,"abs_pos_");
        strcat(tmp_string, tmp2_string);
        check_config(tmp_string, &(rb_obj.reader_cnf.abs_pos[j]));

        strcpy(tmp_string,"current_joints_");
        strcat(tmp_string, tmp2_string);
        check_config(tmp_string, &(rb_obj.reader_cnf.current_joints[j]));

        if (j<6)
        {
            strcpy(tmp_string,"force_");
            strcat(tmp_string, tmp2_string);
            check_config(tmp_string, &(rb_obj.reader_cnf.force[j]));

            strcpy(tmp_string,"desired_force_");
            strcat(tmp_string, tmp2_string);
            check_config(tmp_string, &(rb_obj.reader_cnf.desired_force[j]));

            strcpy(tmp_string,"filtered_force_");
            strcat(tmp_string, tmp2_string);
            check_config(tmp_string, &(rb_obj.reader_cnf.filtered_force[j]));

            strcpy(tmp_string,"current_kartez_position_");
            strcat(tmp_string, tmp2_string);
            check_config(tmp_string, &(rb_obj.reader_cnf.current_kartez_position[j]));

            strcpy(tmp_string,"real_kartez_position_");
            strcat(tmp_string, tmp2_string);
            check_config(tmp_string, &(rb_obj.reader_cnf.real_kartez_position[j]));

            strcpy(tmp_string,"real_kartez_vel_");
            strcat(tmp_string, tmp2_string);
            check_config(tmp_string, &(rb_obj.reader_cnf.real_kartez_vel[j]));

            strcpy(tmp_string,"real_kartez_acc_");
            strcat(tmp_string, tmp2_string);
            check_config(tmp_string, &(rb_obj.reader_cnf.real_kartez_acc[j]));
        }
    }

    // ustawienie priorytetu watku
    set_thread_priority(pthread_self() , MAX_PRIORITY-10);

    // alokacja pamieci pod lokalny bufor z pomiarami
    r_measptr = new reader_data[nr_of_samples];

    // powolanie kanalu komunikacyjnego do odbioru pulsow sterujacych
    if ((my_attach = name_attach(NULL, master->config.return_attach_point_name(configurator::CONFIG_SERVER, "reader_attach_point"),
                                 NAME_FLAG_ATTACH_GLOBAL)) == NULL)
    {// by Y komuniakicja pomiedzy ui i reader'em rozwiazalem poprzez pulsy
        e = errno;
        perror("Failed to attach pulse chanel for READER\n");
        master->msg->message ("Failed to attach pulse chanel for READER");
        //  throw MP_main_error(SYSTEM_ERROR, (uint64_t) 0);
    }

    // GLOWNA PETLA Z OCZEKIWANIEM NA ZLECENIE POMIAROW
    for (;;)
    {

        msr_nr=0; // wyzerowanie liczby pomiarow
        przepelniony=0; // bufor narazie nie jest przepelniony

        int rcvid;

        wyjscie=0; // okresla czy odebrano juz puls rozpoczecia pomiarow

        // dopoki nie przyjdzie puls startu
        while (!wyjscie)
        {
            rcvid = MsgReceive(my_attach->chid, &ui_msg, sizeof(ui_msg), NULL);

            if (rcvid == -1)
            {/* Error condition, exit */
                perror("blad receive w reader\n");
                break;
            }

            if (rcvid == 0)
            {/* Pulse received */
                //  printf("reader puls\n");
                switch (ui_msg.hdr.code)
                {
                case _PULSE_CODE_DISCONNECT:
                    ConnectDetach(ui_msg.hdr.scoid);
                    break;
                case _PULSE_CODE_UNBLOCK:
                    break;
                default:
                    if (ui_msg.hdr.code==READER_START)
                    { // odebrano puls start
                        wyjscie++;
                    }
                }
                continue;
            }

            /* A QNX IO message received, reject */
            if (ui_msg.hdr.type >= _IO_BASE && ui_msg.hdr.type <= _IO_MAX)
            {
                MsgReply(rcvid, EOK, 0, 0);
                continue;
            }
            /* A message (presumable ours) received, handle */
            printf("reader server receive strange message of type: %d\n", ui_msg.data);
            MsgReply(rcvid, EOK, 0, 0);
            rcvid = MsgReceive(my_attach->chid, &ui_msg, sizeof(ui_msg), NULL);

        }

        master->msg->message("measures started");
        set_thread_priority(pthread_self() , MAX_PRIORITY+1);

        rb_obj.reader_wait_for_new_step();
        // dopoki nie przyjdzie puls stopu
        do
        {
            // czekamy na opuszcenie semafora przez watek EDP_SERVO (co mikrokrok)
            rb_obj.reader_wait_for_new_step();

            // sekcja krytyczna odczytu danych pomiarowych dla biezacego kroku
            rb_obj.lock_mutex();

            if (ui_trigger)
            {
                rb_obj.step_data.ui_trigger = 1;
            }
            else
            {
                rb_obj.step_data.ui_trigger = 0;
            }

            // przepisanie danych dla biezacego kroku do bufora lokalnego reader
            memcpy( &(r_measptr[msr_nr]), &rb_obj.step_data, sizeof(reader_data) );

            rb_obj.unlock_mutex();

            // wykrycie przepelnienia
            if ((++msr_nr)>=nr_of_samples)
            {
                msr_nr=0;
                przepelniony=1;
            }

            // warunkowy odbior pulsu (o ile przyszedl)
            stop_event.sigev_notify = SIGEV_UNBLOCK;
            TimerTimeout(CLOCK_REALTIME, _NTO_TIMEOUT_RECEIVE,  &stop_event, NULL, NULL ); // czekamy na odbior pulsu stopu
            rcvid = MsgReceive(my_attach->chid, &ui_msg, sizeof(ui_msg), NULL);

            msg_return=-1;
            ui_trigger = false;

            if (rcvid == -1)
            {/* Error condition, exit */
                // perror("blad receive w reader\n");
            }

            if (rcvid == 0)
            {/* Pulse received */
                // printf("reader puls\n");
                switch (ui_msg.hdr.code)
                {
                case _PULSE_CODE_DISCONNECT:
                    ConnectDetach(ui_msg.hdr.scoid);
                    break;
                case _PULSE_CODE_UNBLOCK:
                    break;
                default:
                    if (ui_msg.hdr.code==READER_STOP)
                    { // odebrano stop
                        msg_return = 0; // dostalismy puls stopu
                    }
                    if (ui_msg.hdr.code==READER_TRIGGER)
                    { // odebrano stop
                        ui_trigger = true;
                    }

                }
            }

            if (rcvid > 0)
            {
                /* A QNX IO message received, reject */
                if (ui_msg.hdr.type >= _IO_BASE && ui_msg.hdr.type <= _IO_MAX)
                {
                    MsgReply(rcvid, EOK, 0, 0);
                }
                else
                {
                    /* A message (presumable ours) received, handle */
                    printf("reader server receive strange message of type: %d\n", ui_msg.data);
                    MsgReply(rcvid, EOK, 0, 0);
                }
            }
        }
        while(msg_return==-1); // dopoki nie przyjdzie puls stopu


        set_thread_priority(pthread_self() , 1);// Najnizszy priorytet podczas proby zapisu do pliku
        master->msg->message("measures stopped");

        // przygotowanie nazwy pliku do ktorego beda zapisane pomiary
        time_of_day = time( NULL );
        strftime( file_date, 40, "%g%m%d_%H-%M-%S", localtime( &time_of_day ) );

        sprintf(file_name,"/%s_%s_pomiar-%d", file_date, robot_name, ++file_counter);
        strcpy(config_file_with_dir, reader_meassures_dir);

        strcat(config_file_with_dir,file_name);

        ofstream outfile(config_file_with_dir, ios::out);
        if (!outfile)  // jesli plik nie instnieje
        {
            std::cerr << "Cannot open file: " << file_name << '\n';
            perror("because of");
            master->msg->message("cannot open destination file");
        } else
        { // jesli plik istnieje

            // sprawdzenie czy bufor byl przepelniony i odpowiednie przygotowanie granic bufora przy zapi sie do pliku
            if (przepelniony)
            {
                k=msr_nr;
                msr_counter=nr_of_samples;
            }
            else
            {
                k=0;
                msr_counter=msr_nr;
            }

            // dla calego horyzontu pomiarow

            for (i = 0; i < msr_counter; i++)
            {

                if ( k == nr_of_samples)
                    k = 0;

                // zapis pomiarow z biezacego kroku do pliku

                outfile << r_measptr[k].step << " ";
                if (rb_obj.reader_cnf.msec)
                    outfile << r_measptr[k].msec << " ";
                if (rb_obj.reader_cnf.servo_tryb)
                    outfile << r_measptr[k].servo_tryb << " ";

                for (int j=0; j<MAX_SERVOS_NR; j++)
                {
                    if (rb_obj.reader_cnf.desired_inc[j])
                        outfile << r_measptr[k].desired_inc[j] << " ";
                    if (rb_obj.reader_cnf.current_inc[j])
                        outfile << r_measptr[k].current_inc[j] << " ";
                    if (rb_obj.reader_cnf.pwm[j])
                        outfile << r_measptr[k].pwm[j] << " ";
                    if (rb_obj.reader_cnf.uchyb[j])
                        outfile << r_measptr[k].uchyb[j] << " ";
                    if (rb_obj.reader_cnf.abs_pos[j])
                        outfile << r_measptr[k].abs_pos[j] << " ";
                }

                outfile << "j: ";

                for (int j=0; j<MAX_SERVOS_NR; j++)
                {
                    if (rb_obj.reader_cnf.current_joints[j])
                        outfile << r_measptr[k].current_joints[j] << " ";
                }

                outfile << "f: ";

                for (int j=0; j<6; j++)
                {
                    if (rb_obj.reader_cnf.force[j])
                        outfile << r_measptr[k].force[j] << " ";
                    if (rb_obj.reader_cnf.desired_force[j])
                        outfile << r_measptr[k].desired_force[j] << " ";
                    if (rb_obj.reader_cnf.filtered_force[j])
                        outfile << r_measptr[k].filtered_force[j] << " ";
                }


                outfile << "k: ";

                for (int j=0; j<6; j++)
                {
                    if (rb_obj.reader_cnf.current_kartez_position[j])
                        outfile << r_measptr[k].current_kartez_position[j] << " ";
                }

                outfile << "r: ";

                for (int j=0; j<6; j++)
                {
                    if (rb_obj.reader_cnf.real_kartez_position[j])
                        outfile << r_measptr[k].real_kartez_position[j] << " ";
                }

                outfile << "v: ";

                for (int j=0; j<6; j++)
                {
                    if (rb_obj.reader_cnf.real_kartez_vel[j])
                        outfile << r_measptr[k].real_kartez_vel[j] << " ";
                }

                outfile << "a: ";

                for (int j=0; j<6; j++)
                {
                    if (rb_obj.reader_cnf.real_kartez_acc[j])
                        outfile << r_measptr[k].real_kartez_acc[j] << " ";
                }

                outfile << "t: " << r_measptr[k].ui_trigger;

                outfile << '\n';

                k++;
            } // end for(i = 0; i < msr_counter; i++)

            // zamkniecie pliku
            outfile.close();
            master->msg->message("file writing is finished");
        }

        set_thread_priority(pthread_self() , MAX_PRIORITY-10);

    } // end: for (;;)

    delete[] robot_name;
    delete[] reader_meassures_dir;
};
