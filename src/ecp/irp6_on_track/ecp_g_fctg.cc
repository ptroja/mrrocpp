// -------------------------------------------------------------------------
// Proces:		EFFECTOR CONTROL PROCESS (ECP)
// Plik:			ecp_fctg.cc
// System:	QNX/MRROC++  v. 6.3
// Opis:		force_controlled_trajectory_generator - definicja metod klasy
// 				-> poruszanie z UI (lewo - prawo)
// 				-> generator do uczenia trajektorii
// 				-> czujnik sily do kontorlli wykonywanego ruchu
// Autor:		tkornuta
// Data:		28.11.2005
// -------------------------------------------------------------------------

/********************************* INCLUDES *********************************/
#include <stdio.h>
#include <string.h>
#include <fstream>
#include <pthread.h>
#include <math.h>

#include "common/typedefs.h"
#include "common/impconst.h"
#include "common/com_buf.h"

#include "lib/srlib.h"

// Naglowek z klasa czujnikow ecp_mp_force_sensor.
#include "ecp_mp/ecp_mp_s_force.h"

#include "ecp/irp6_on_track/ecp_local.h"
#include "ecp/irp6_on_track/ecp_g_fctg.h"

// Mutex do wykonywania odczytow z czujnika.
//pthread_mutex_t FORCE_SENSOR_READINGS_MUTEX = PTHREAD_MUTEX_INITIALIZER;

/*********************** RETURN SENSOR READING ***************************/
void force_controlled_trajectory_generator::return_sensor_reading(ecp_mp::sensor::force& the_sensor, double sensor_reading[6]){
	SENSOR_IMAGE si;
	// Nowe odczyty czujnika.
	the_sensor.get_reading(&si);
	// Przepisanie pozycji z bufora.
	memcpy(sensor_reading, si.sensor_union.force.rez, 6*sizeof(double));
    }; // end: return_sensor_reading

/*************************** RETURN  POSITION *******************************/
void force_controlled_trajectory_generator::return_position (double robot_position[8]){
    // Sekcja krytyczna.
    pthread_mutex_lock(&ROBOT_POSITION_MUTEX);
        // Przepisanie pozycji z bufora.
        memcpy(robot_position, current_position, 8*sizeof(double));

/*        printf("zwracam current position:\n");
        for(int i=0; i<8; i++)
            printf("%f\t", current_motor_position[i]);
        printf("\n");*/

        // Koniec sekcji krytycznej.
    pthread_mutex_unlock(&ROBOT_POSITION_MUTEX);
    }; // end: return_position

/************************ RETURN  MOTOR POSITION ***************************/
void force_controlled_trajectory_generator::return_motor_position (double robot_position[8]){
    // Sekcja krytyczna.
    pthread_mutex_lock(&ROBOT_POSITION_MUTEX);
        // Przepisanie pozycji z bufora.
        memcpy(robot_position, current_motor_position, 8*sizeof(double));
    // Koniec sekcji krytycznej.
    pthread_mutex_unlock(&ROBOT_POSITION_MUTEX);
    }; // end: return_position

/************************* GET CURRENT POSITION *****************************/
void force_controlled_trajectory_generator::get_current_position (){
    // Sekcja krytyczna.
    pthread_mutex_lock(&ROBOT_POSITION_MUTEX);
        // Odczytanie polozenia robota
        // Przygotowanie rozkazu dla EDP.
        the_robot->EDP_data.instruction_type = GET;
        the_robot->EDP_data.get_type = ARM_DV; // ARM
        the_robot->EDP_data.get_arm_type = current_control;
        // Przepisanie rozkazu do bufora wysylkowego.
        the_robot->create_command();
        // Zlecenie ruchu robota.
        the_robot->execute_motion();
        // Odebranie danych.
        the_robot->get_reply();
        // Przepisanie obecnego polozenia robota do bufora.
        if (current_control == MOTOR){
            memcpy(current_position, the_robot->EDP_data.current_motor_arm_coordinates, 8*sizeof(double));
            memcpy(current_motor_position, the_robot->EDP_data.current_motor_arm_coordinates, 8*sizeof(double));
            }
        else if (current_control == XYZ_EULER_ZYZ){
            memcpy(current_position, the_robot->EDP_data.current_XYZ_ZYZ_arm_coordinates, 6*sizeof(double));
            // Odczytanie polowenia na motorach.
            // Przygotowanie rozkazu dla EDP.
            the_robot->EDP_data.instruction_type = GET;
            the_robot->EDP_data.get_type = ARM_DV; // ARM
            the_robot->EDP_data.get_arm_type = MOTOR;
            // Przepisanie rozkazu do bufora wysylkowego.
            the_robot->create_command();
            // Zlecenie ruchu robota.
            the_robot->execute_motion();
            // Odebranie danych.
            the_robot->get_reply();
            memcpy(current_motor_position, the_robot->EDP_data.current_motor_arm_coordinates, 8*sizeof(double));
            } // end: if*/
    // Koniec sekcji krytycznej.
    pthread_mutex_unlock(&ROBOT_POSITION_MUTEX);
    }; // end: get_current_position


/**************************** CHANGE CONTROL *******************************/
void force_controlled_trajectory_generator::change_control(POSE_SPECIFICATION ps){
    // Jesli nic nie trzeba zmienic
    if (current_control == ps)
        return;
    // Zmiana sterowania.
    if (ps == MOTOR){
        // Zmiana na sterowanie na silnikach.
        current_control = MOTOR;
        // Przepisanie zmiennych.
        memcpy(current_delta, motor_delta, 8*sizeof(double));
        memcpy(current_delta_increment, motor_delta_increment, 8*sizeof(double));
        memcpy(current_max_delta_increment, motor_max_delta_increment, 8*sizeof(double));
        }
    else if (ps == XYZ_EULER_ZYZ){
        // Zmiana na sterowanie we wspolrzednych zewnetrznych.
        current_control = XYZ_EULER_ZYZ;
        // Przepisanie zmiennych.
        memcpy(current_delta, external_delta, 6*sizeof(double));
        memcpy(current_delta_increment, external_delta_increment, 6*sizeof(double));
        memcpy(current_max_delta_increment, external_max_delta_increment, 6*sizeof(double));
        };
    }; // end: change_control


/*****************************  KONSTRUKTOR *********************************/
force_controlled_trajectory_generator::force_controlled_trajectory_generator (ecp_task& _ecp_task)
	:ecp_teach_in_generator(_ecp_task)
{
	pthread_mutex_init(&ROBOT_POSITION_MUTEX, NULL);
    // Puste listy.
//    pose_list_ptr = NULL;
//    current_pose_ptr = NULL;
    position_list.clear();
    // Dane dla motorow.
    // Wypelnienie przesuniecia na motorach.
    motor_delta[0] = 24;
    motor_delta[1] = -10;
    motor_delta[2] = 14;
    motor_delta[3] = 10;
    motor_delta[4] = -16;
    motor_delta[5] = 16;
    motor_delta[6] = 0;
    motor_delta[7] = 0;
    // Przyrost przesuniecia na motorach.
    motor_delta_increment[0] = 1;
    motor_delta_increment[1] = 0.5;
    motor_delta_increment[2] = 0.5;
    motor_delta_increment[3] = 0.5;
    motor_delta_increment[4] = 0.5;
    motor_delta_increment[5] = 0.5;
    motor_delta_increment[6] = 0;
    motor_delta_increment[7] = 0;
    // Maksymalny przyrost przesuniecia na motorach.
    motor_max_delta_increment[0] = 4;
    motor_max_delta_increment[1] = 2;
    motor_max_delta_increment[2] = 2;
    motor_max_delta_increment[3] = 2;
    motor_max_delta_increment[4] = 2;
    motor_max_delta_increment[5] = 2;
    motor_max_delta_increment[6] = 0;
    motor_max_delta_increment[7] = 0;
    // Dane dla wspolrzednych zewnetrznych.
    // Wypelnienie przesuniecia na wspolrzednych zewnetrznych.
    external_delta[0] = 0.02;
    external_delta[1] = 0.02;
    external_delta[2] = 0.02;
    external_delta[3] = 0.03;
    external_delta[4] = -0.1;
    external_delta[5] = 0.1;
    // Przyrost przesuniecia na wspolrzednych zewnetrznych.
    external_delta_increment[0] = 0.001;
    external_delta_increment[1] = 0.001;
    external_delta_increment[2] = 0.001;
    external_delta_increment[3] = 0.001;
    external_delta_increment[4] = 0.005;
    external_delta_increment[5] = 0.005;
    // Maksymalny przyrost przesuniecia na wspolrzednych zewnetrznych.
    external_max_delta_increment[0] = 0.003;
    external_max_delta_increment[1] = 0.003;
    external_max_delta_increment[2] = 0.003;
    external_max_delta_increment[3] = 0.003;
    external_max_delta_increment[4] = 0.02;
    external_max_delta_increment[5] = 0.02;
    // Chwilowe ustawienie rodzaju sterowania.
    current_control = XYZ_EULER_ZYZ;
    // Ustawienie sterowania na MOTOR.
    change_control(MOTOR);
    }; // end: force_controlled_trajectory_generator


/********************************* ADD STEP ***********************************/
void force_controlled_trajectory_generator::add_step (int motion_time){
    // Sekcja krytyczna.
    pthread_mutex_lock(&ROBOT_POSITION_MUTEX);
    // Dodanie elementu do listy
    if (!pose_list.empty()){
        // Jesli glowa pusta.
        create_pose_list_head(MOTOR, motion_time, current_motor_position);
    }else{
        // Jesli nastepny element.
        insert_pose_list_element(MOTOR, motion_time, current_motor_position);
        }; // end else
    // Wyswietlenie dodanego elementu.
/*    printf("Robot :: ");
    for(int i=0; i<8; i++)
        printf("%f\t", current_motor_position[i]);
    printf("\n");*/
  // Koniec sekcji krytycznej.
    pthread_mutex_unlock(&ROBOT_POSITION_MUTEX);
    }; // end: add_step

/****************************** SET MOVE TYPE *********************************/
void force_controlled_trajectory_generator::set_move_type(short move_type){
    if (move_type>0){
        // Ruch w "prawo".
        dir = 1;
        number = move_type - 1;
    }else{
        // Ruch w "lewo".
        dir = -1;
        number = -move_type - 1;
        };
    }; // end: set_move_type

/******************************** FIRST STEP ***********************************/
bool force_controlled_trajectory_generator::first_step(){
    // Pozycja robota.
    double robot_position[8];
    // Pozycja - tymczasowa zmienna.
    double tmp_position;
    // Pozycja docelowa.
    double start_position;
    // Pozycja docelowa.
    double stop_position;
    // Pozycja po przyspieszeniu.
    double after_acceleration_position;
    // Przyrost na motorach.
    double tmp_delta;
    // Czujnik sily.
    ecp_mp::sensor::force* the_sensor;
    // Jesli lista czujnikow nie jest pusta.
    if (sensor_m.size()>0){
        the_sensor = (ecp_mp::sensor::force*)((sensor_m.begin())->second);
    }else{
        the_sensor = NULL;
    }
    // Jesli mamy uzywac czujnika sily.
    if (the_sensor){
        // Sprawdzenie odczytow czujnika sily.
        check_force_condition(*the_sensor);
    }

    // Pobranie obecnej pozycji robota.
    return_position(robot_position);
    // Obecna pozycja robota.
    start_position = robot_position[number];
    // Docelowa pozycja koncowa.
    stop_position = start_position + dir*current_delta[number];
    // Pierwsza faza - przyspieszenie.
    tmp_delta = 0;
    increment_delta(tmp_delta, dir*current_delta[number], current_max_delta_increment[number], current_delta_increment[number]);
    // Przesuniecie sie o delte.
    tmp_position = start_position + tmp_delta;
    // Oproznienie listy.
    flush_position_list();
    // Zapamietanie pozycji - stworzenie glowy.
    create_position_list_head(tmp_position);
    // Przyspieszanie, dopoki nie osiagnieto maksymalnego przyrostu.
    while(increment_delta(tmp_delta, dir*current_delta[number], current_max_delta_increment[number], current_delta_increment[number])){
        // Przesuniecie sie o delte.
        tmp_position = tmp_position + tmp_delta;
        // Zapamietanie nastepnej pozycji.
        insert_position_list_element(tmp_position);
	}
    // Pozycja, w ktorej nalezy zaczac zwalniac.
    after_acceleration_position = tmp_position;
    // Faza ruchu ze stalym przesunieciem.
    while(slow_down_condition(start_position, after_acceleration_position, tmp_position, stop_position) == false){
        // Przesuniecie sie o delte.
        tmp_position = tmp_position + tmp_delta;
        // Zapamietanie nastepnej pozycji.
        insert_position_list_element(tmp_position);
	}
    // Ostatnia faza - zwalnianie.
    while(decrement_delta(tmp_delta, dir*current_delta[number], current_delta_increment[number])){
        // Przesuniecie sie o delte.
        tmp_position = tmp_position + tmp_delta;
        // Zapamietanie nastepnej pozycji.
        insert_position_list_element(tmp_position);
	}
    // Przygotowanie rozkazu dla EDP.
    the_robot->EDP_data.instruction_type = SET;
    the_robot->EDP_data.set_type = ARM_DV; // ARM
    the_robot->EDP_data.set_arm_type = current_control;
    the_robot->EDP_data.motion_type = ABSOLUTE;
     the_robot->EDP_data.next_interpolation_type = MIM;
    the_robot->EDP_data.motion_steps = 70;
    the_robot->EDP_data.value_in_step_no = the_robot->EDP_data.motion_steps-5;

    // Zerowe przesuniecie na innych osiach.
    if (current_control == MOTOR)
        memcpy(the_robot->EDP_data.next_motor_arm_coordinates, robot_position, 8*sizeof(double));
    // Zerowe przesuniecie robota do danej pozycji - wspolrzedne zewnetrzne.
    else if (current_control == XYZ_EULER_ZYZ)
        memcpy(the_robot->EDP_data.next_XYZ_ZYZ_arm_coordinates, robot_position, 6*sizeof(double));
    // Przesuniecie sie na pierwszy element listy.
    initiate_position_list();
    if(!is_position_list_element()){
        // Pusta tajektoria.
        return false;
	}
    // Pobranie pierwszej pozycji.
    get_position_list_element(tmp_position);
    // Przesuniecie robota do pierwszej pozycji - na motorach.
    if (current_control == MOTOR)
        the_robot->EDP_data.next_motor_arm_coordinates[number] = tmp_position;
    // Przesuniecie robota do pierwszej pozycji - wspolrzedne zewnetrzne.
    if (current_control == XYZ_EULER_ZYZ)
        the_robot->EDP_data.next_XYZ_ZYZ_arm_coordinates[number] = tmp_position;
    // Przepisanie rozkazu do bufora wysylkowego.

    // Przesuniecie na nastepny element z listy.
    next_position_list_element();
    // Trajektoria przygotowana.
    return true;
}

/******************************** NEXT STEP ***********************************/
bool force_controlled_trajectory_generator::next_step ( ) {
    // Pozycja - tymczasowa zmienna.
    double tmp_position;
    // Czujnik sily.
    ecp_mp::sensor::force* the_sensor;
    // Jesli lista czujnikow nie jest pusta.
    if (sensor_m.size()>0){
        the_sensor = (ecp_mp::sensor::force*)((sensor_m.begin())->second);
        // Sprawdzenie odczytow czujnika sily.
        	check_force_condition(*the_sensor);
	}

    // Sprawdzenie, czy nie wykonano calej trajektorii.
    if(!is_position_list_element()){
        // Oproznienie listy.
        flush_position_list();
        // Koniec ruchu.
        return false;
	}

    // Pobranie elementu z listy.
    get_position_list_element(tmp_position);
    // Przesuniecie robota do danej pozycji - na motorach.
    if (current_control == MOTOR)
        the_robot->EDP_data.next_motor_arm_coordinates[number] = tmp_position;
    // Przesuniecie robota do danej pozycji - wspolrzedne zewnetrzne.
    if (current_control == XYZ_EULER_ZYZ)
        the_robot->EDP_data.next_XYZ_ZYZ_arm_coordinates[number] = tmp_position;
    // Przepisanie rozkazu do bufora wysylkowego.

    // Przesuniecie na nastepny element z listy.
    next_position_list_element();
    // Nie wykonano jeszcze calego ruchu.
    return true;
}

/*********************  CHECK FORCE CONDITION **************************/
void force_controlled_trajectory_generator::check_force_condition(ecp_mp::sensor::force& the_sensor){
    double tmp_reading[6];
    // Pobranie odczytow z czujnika sily.
    return_sensor_reading(the_sensor, tmp_reading);
    // Sprawdzenie, czy nie wystapila za duza sila.
    for (int i=0; i<6; i++)
        if (fabs(tmp_reading[i]) > dangerous_force)
                throw ECP_error(NON_FATAL_ERROR, DANGEROUS_FORCE_DETECTED);
    // Sila w porzadku.
}

/***************************  INCREMENT DELTA ********************************/
bool force_controlled_trajectory_generator::increment_delta(double &tmp_delta, double direction, double max_delta_increment, double delta_increment){
    // Sprawdzenie znaku maksymalnego przerostu.
    if (direction < 0){
        // Zmniejszenie przyrostu.
        tmp_delta -= delta_increment;
        // Sprawdzenie, czy osiagnieta zostala "maksymalna" wielkosc przystostu.
        if ((-tmp_delta) >= max_delta_increment)
            return false;
    }else{
        // Zwiekszenie przyrostu.
        tmp_delta += delta_increment;
        // Sprawdzenie, czy osiagnieta zostala "maksymalna" wielkosc przystostu.
        if (tmp_delta >= max_delta_increment)
            return false;
        }; // end: else
    // Nie osiagnieto jeszcze maksymalnego przyspieszenia.
    return true;
}

/*************************  SLOW DOWN CONDITION ****************************/
bool force_controlled_trajectory_generator::slow_down_condition(double start_position, double after_acceleration_position, double current_position, double stop_position){
    // Jesli kierunek ruchu ma znak dodatni.
    if (start_position < stop_position){
        // Jezeli pozostala tylko droga na zwolnienie.
        if ((current_position + (after_acceleration_position - start_position)) >= stop_position)
            return true;
        else
            return false;
    }else{
        // Jezeli pozostala tylko droga na zwolnienie.
        if ((current_position - (start_position - after_acceleration_position)) <= stop_position)
            return true;
        else
            return false;
        }; // end: else
    }; // end: check_slow_down_condition

/***************************  DECREMENT DELTA *******************************/
bool force_controlled_trajectory_generator::decrement_delta(double &tmp_delta, double direction, double delta_increment){
    // Sprawdzenie kierunku.
    if (direction > 0){
        // Zmniejszenie przyrostu.
        tmp_delta -= delta_increment;
        // Sprawdzenie, czy robot sie nie zatrzymal.
        if (tmp_delta < 0)
            return false;
    }else{
        // Zwiekszenie przyrostu.
        tmp_delta += delta_increment;
        // Sprawdzenie, czy robot sie nie zatrzymal.
        if (tmp_delta > 0)
            return false;
        }; // end: else
    // Wykonanie dalszego ruchu.
    return true;
    }; // end: decrement_delta

/**************************** SAVE TRAJECTORY ********************************/
void force_controlled_trajectory_generator::save_trajectory(char* filename) {
    // Pozycja robota.
    ecp_taught_in_pose tip;
    int j;
    if (pose_list_length() == 0){
        sr_ecp_msg.message("Empty trajectory");
        return;
        };
try{
    // Otwarcie pliku.
	std::ofstream to_file(filename);
    if (!to_file)
        throw ECP_error(FATAL_ERROR, SAVE_FILE_ERROR);
    // Przejscie na poczatek listy.
    initiate_pose_list();
    // Rodzaj wspolrzednych.
    to_file << "MOTOR\n";
    // Liczba elementow.
    to_file << pose_list_length() << '\n';
    // Zapisywanie kolejnych pozycji.
    while (!is_last_list_element()){
        // Pobranie pozycji.
        get_pose (tip);
        // Zapis czasu trawnia kroku do pliku.
        to_file << tip.motion_time << ' ';
        // Zapis pozycji.
        for (j = 0; j < 8; j++)
            to_file << tip.coordinates[j] << ' ';
        // Nastepna linia.
        to_file << '\n';
        // Nastepna pozycja.
        next_pose_list_ptr();
        };
    // Zapisanie ostatniego elementu.
    // Pobranie pozycji.
    get_pose (tip);
    // Zapis czasu trawnia kroku do pliku.
    to_file << tip.motion_time << ' ';
    // Zapis pozycji.
    for (j = 0; j < 8; j++)
        to_file << tip.coordinates[j] << ' ';
    // Nastepna linia.
    to_file << '\n';
    // Zamkniecie pliku.
    to_file.close();
    sr_ecp_msg.message("Trajectory saved properly");
    } // end: TRY
catch(ECP_error e){
    // Wylapanie i oblsuga bledow.
    sr_ecp_msg.message (e.error_class, e.error_no);
    };
    }; // end: save_trajectory


/*************************  SET DANGEROUS FORCE *****************************/
void force_controlled_trajectory_generator::set_dangerous_force(){
	dangerous_force = ecp_t.config.return_int_value("dangerous_force");
	sr_ecp_msg.message("Dangerous force size readed properly from INI file");
    }; // end: set_dangerous_force



/******************************  DESTRUKTOR **********************************/
force_controlled_trajectory_generator::~force_controlled_trajectory_generator (void) {
    // Usuniecie elementow z obu list.
    flush_pose_list();
    flush_position_list();
	pthread_mutex_destroy(&ROBOT_POSITION_MUTEX);
    };

/********************* METODY ZWIAZANE Z LISTA POZYCJI **********************/
void force_controlled_trajectory_generator::flush_position_list ( void ) {
    // Jezeli sa jakies elementy
    position_list.clear();
    }; // end: flush_position_list

void force_controlled_trajectory_generator::initiate_position_list(void) {
    position_list_iterator = position_list.begin();
    };

void force_controlled_trajectory_generator::next_position_list_element (void) {
    // Przejscie na nastepny element.
  	position_list_iterator++;
    };

void force_controlled_trajectory_generator::get_position_list_element (double &position){
    // Przepisanie polozenia z listy.
    position = *position_list_iterator;
    };


bool force_controlled_trajectory_generator::is_position_list_element ( void ) {
    // sprawdza czy element nie jest NULL.
    if (position_list_iterator != position_list.end())
        return true;
    else
        return false;
    };

void force_controlled_trajectory_generator::create_position_list_head (double position) {
    // Wstawienie glowy.
    	position_list.push_back(double(position));
	position_list_iterator = position_list.begin();
    };

void force_controlled_trajectory_generator::insert_position_list_element (double position) {
    // Wlasciwe wstawienie elementu.
    position_list.push_back(double(position));
	position_list_iterator++;
    };

/**************** KONIEC: METODY ZWIAZANE Z LISTA POZYCJI ******************/
