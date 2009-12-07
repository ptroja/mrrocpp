// -------------------------------------------------------------------------
// Proces:		EFFECTOR CONTROL PROCESS (lib::ECP)
// Plik:			ecp_trg.cc
// System:	QNX/MRROC++  v. 6.3
// Opis:		trajectory_reproduce_generator - definicja metod klasy
// 				generator do odtwarzania trajektorii i odczytywania pozycji z linialow po ruchu robota
// Autor		tkornuta
// Data:		28.11.2005 -
// -------------------------------------------------------------------------

/********************************* INCLUDES *********************************/
#include <ctype.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <errno.h>
#include <fstream>

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "lib/srlib.h"

#include "ecp/irp6_on_track/ecp_r_irp6ot.h"

#include "ecp/irp6_on_track/ecp_g_trg.h"
#include "ecp_mp/ecp_mp_s_digital_scales.h"
#include "ecp_mp/ecp_mp_s_force.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot {
namespace generator {

/********************************* GLOBALS **********************************/

/************************* GET CURRENT POSITION *****************************/
void trajectory_reproduce::get_current_position(double current_position[6]){
    // Odczytanie polozenia robota
    // Przygotowanie rozkazu dla EDP.
    the_robot->ecp_command.instruction.instruction_type = lib::GET;
    the_robot->ecp_command.instruction.get_type = ARM_DV; // ARM
    // Sprawdzenie rodzaju ramienia.
    the_robot->ecp_command.instruction.get_arm_type = lib::MOTOR;
    // Przepisanie rozkazu do bufora wysylkowego.

    // Zlecenie ruchu robota.
    the_robot->execute_motion();
    // Odebranie danych.

    // Przepisanie obecnego polozenia robota do bufora w zaleznosci od rodzaju wspolrzednych.
    memcpy(current_position, the_robot->reply_package.arm.pf_def.arm_coordinates, MAX_SERVOS_NR*sizeof(double));
    } // end: get_current_position

/******************** PREPARE GENERATOR FOR MOTION  ************************/
void trajectory_reproduce::prepare_generator_for_motion(void){
    // Poczatek listy.
    initiate_pose_list();
    // Wyczyszczenie listy z pozycjami posrednimi.
    flush_interpose_list();
    } // end: prepare_generator_for_motion

/*********************** CREATE COMMAND FOR POSE **************************/
void trajectory_reproduce::create_command_for_pose(common::ecp_taught_in_pose& tip) {
// printf("create_command_for_pose: czas %f | %f, %f, %f, %f, %f, %f\n", tip.motion_time,
// tip.coordinates[0], tip.coordinates[1], tip.coordinates[2], tip.coordinates[3], tip.coordinates[4], tip.coordinates[5]);
    // sprawdzenie rodzaju wspolrzednych
    switch ( tip.arm_type ) {
    case lib::C_MOTOR:
        the_robot->ecp_command.instruction.instruction_type = lib::SET;
        the_robot->ecp_command.instruction.set_type = ARM_DV; // ARM
        the_robot->ecp_command.instruction.set_arm_type = lib::MOTOR;
        the_robot->ecp_command.instruction.motion_type = lib::ABSOLUTE;
         the_robot->ecp_command.instruction.interpolation_type = lib::MIM;
        the_robot->ecp_command.instruction.motion_steps = (uint16_t) ceil(tip.motion_time/STEP);
        the_robot->ecp_command.instruction.value_in_step_no = the_robot->ecp_command.instruction.motion_steps;
        memcpy (the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates, tip.coordinates, IRP6_ON_TRACK_NUM_OF_SERVOS*sizeof (double));
        break;
    case lib::C_JOINT:
        the_robot->ecp_command.instruction.instruction_type = lib::SET;
        the_robot->ecp_command.instruction.set_type = ARM_DV; // ARM
        the_robot->ecp_command.instruction.set_arm_type = lib::JOINT;
        the_robot->ecp_command.instruction.motion_type = lib::ABSOLUTE;
         the_robot->ecp_command.instruction.interpolation_type = lib::MIM;
        the_robot->ecp_command.instruction.motion_steps = (uint16_t) ceil(tip.motion_time/STEP);
        the_robot->ecp_command.instruction.value_in_step_no = the_robot->ecp_command.instruction.motion_steps;
        memcpy (the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates, tip.coordinates, IRP6_ON_TRACK_NUM_OF_SERVOS*sizeof (double));
        break;
    case lib::C_XYZ_EULER_ZYZ:
        the_robot->ecp_command.instruction.instruction_type = lib::SET;
        the_robot->ecp_command.instruction.set_type = ARM_DV; // ARM
        the_robot->ecp_command.instruction.set_arm_type = lib::XYZ_EULER_ZYZ;
        the_robot->ecp_command.instruction.motion_type = lib::ABSOLUTE;
         the_robot->ecp_command.instruction.interpolation_type = lib::MIM;
        the_robot->ecp_command.instruction.motion_steps = (uint16_t) ceil(tip.motion_time/STEP);
        the_robot->ecp_command.instruction.value_in_step_no = the_robot->ecp_command.instruction.motion_steps;
        memcpy (the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates, tip.coordinates, MAX_SERVOS_NR*sizeof (double));
        break;
    case lib::C_XYZ_ANGLE_AXIS:
        the_robot->ecp_command.instruction.instruction_type = lib::SET;
        the_robot->ecp_command.instruction.set_type = ARM_DV; // ARM
        the_robot->ecp_command.instruction.set_arm_type = lib::XYZ_ANGLE_AXIS;
        the_robot->ecp_command.instruction.motion_type = lib::ABSOLUTE;
         the_robot->ecp_command.instruction.interpolation_type = lib::MIM;
        the_robot->ecp_command.instruction.motion_steps = (uint16_t) ceil(tip.motion_time/STEP);
        the_robot->ecp_command.instruction.value_in_step_no = the_robot->ecp_command.instruction.motion_steps;
        memcpy (the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates, tip.coordinates, MAX_SERVOS_NR*sizeof (double));
        break;
    default:
        break;
        } // end: switch

    } // end: create_command_for_pose


/******************************** FIRST STEP ***********************************/
bool trajectory_reproduce::first_step (){
    // Przygotowanie trajektorii do wykonania.
    // Pozycja poczatkowa.
    double current_position[6];
    // Pozycja koncowa ruchu.
    common::ecp_taught_in_pose tip;
    // Przesuniecie miedzy kolejnymi krokami posrednimi.
    double delta[6];
    // Czas trwania kroku posredniego.
    double interpose_motion_time = 0.01; // = 10 ms
    // Liczba krokow posrednich.
    short interpose_number;
    // Liczniki, petli.
    int  i, step;
	// Sprawdzenie, czy lista czujnikow nie jest pusta.
    if (sensor_m.size() > 0){
	    // Czujnik sily.
    	ecp_mp::sensor::force* the_sensor;
        the_sensor = (ecp_mp::sensor::force*)((sensor_m.begin())->second);
        // Sprawdzenie odczytow czujnika sily.
        check_force_condition(*the_sensor);
        }
    // Jesli juz wykonano wszystkie ruchy.
    if (is_pose_list_element() == false)
        return false;
    // Pobranie pozycji obecnej robota.
    get_current_position(current_position);
    // Pobranie danych o pozycji koncowej.
    get_pose (tip);
    // Podzielenie makrokroku.
    // Obliczenie liczby krokow posrednich.
    interpose_number = (int) ceil (tip.motion_time / interpose_motion_time) - 12;
    // +12, bo +6 (+3+2+1) przyspiesznia i +6 (+3+2+1) zwalnianie)
    // Obliczenie przemieszczenia krokow posrednich.
    for (i=0; i<6; i++)
        delta[i] = (tip.coordinates[i] - current_position[i]) / interpose_number;
    // Wyczyszczenie listy z pozycjami posrednimi.
    flush_interpose_list();
    // FAZA PRZYSPIESZANIA.
    // Pierwsze polozenie posrednie -> przesuniecie o delte.
    for (i=0; i<6; i++)
        current_position[i] += delta[i];
    // Stworzenie glowy listy polozen posrednich.
    create_interpose_list_head(tip.arm_type, interpose_motion_time*4, current_position);
    // Nastepne polozenie posrednie -> przesuniecie o delte.
    for (i=0; i<6; i++)
        current_position[i] += delta[i];
    // Stworzenie bnastepnego elementu listy polozen posrednich.
    insert_interpose_list_element (tip.arm_type, interpose_motion_time*3, current_position);
    // Nastepne polozenie posrednie -> przesuniecie o delte.
    for (i=0; i<6; i++)
        current_position[i] += delta[i];
    // Stworzenie bnastepnego elementu listy polozen posrednich.
    insert_interpose_list_element (tip.arm_type, interpose_motion_time*2, current_position);
    // FAZA ZE STALA PREDKOSCIA.
    // Obliczenie polozen posrednich.
    for (step=3; step< (interpose_number-3); step++){
        // Nastepne polozenie posrednie -> przesuniecie o delte.
        for (i=0; i<6; i++)
            current_position[i] += delta[i];
        // Stworzenie bnastepnego elementu listy polozen posrednich.
        insert_interpose_list_element (tip.arm_type, interpose_motion_time, current_position);
        }
    // FAZA ZWALNIANIA.
    // Nastepne polozenie posrednie -> przesuniecie o delte.
    for (i=0; i<6; i++)
        current_position[i] += delta[i];
    // Stworzenie nastepnego elementu listy polozen posrednich.
    insert_interpose_list_element (tip.arm_type, interpose_motion_time*2, current_position);
    // Nastepne polozenie posrednie -> przesuniecie o delte.
    for (i=0; i<6; i++)
        current_position[i] += delta[i];
    // Stworzenie bnastepnego elementu listy polozen posrednich.
    insert_interpose_list_element (tip.arm_type, interpose_motion_time*3, current_position);
    // Nastepne polozenie posrednie -> przesuniecie o delte.
    for (i=0; i<6; i++)
        current_position[i] += delta[i];
    // Stworzenie bnastepnego elementu listy polozen posrednich.
    insert_interpose_list_element (tip.arm_type, interpose_motion_time*4, current_position);
    // Wykonywanie trajektorii - krokow posrednich.
    // Przesuniecie na pierwsza pozycje posrednia.
    initiate_interpose_list ();
    // Odczytanie pierwszej pozycji posredniej
    get_interpose_list_element(tip);
    // Ruch do pierwszej pozycji posredniej.
    create_command_for_pose(tip);
    // Przesuniecie na nastepny element z listy pozycji posrednich.
    next_interpose_list_element();
    // Mozna wykonac ruch.
    return true;
    } // end: first_step

/******************************** NEXT STEP ***********************************/
bool trajectory_reproduce::next_step (){
    // Wlasciewe wykonywanie trajektorii.
    // Pozycja - tymczasowa zmienna.
	common::ecp_taught_in_pose interpose;
    // Jesli lista czujnikow nie jest pusta.
    if (sensor_m.size() > 0){
	    // Czujnik sily.
    	ecp_mp::sensor::force* the_sensor;
        the_sensor = (ecp_mp::sensor::force*)((sensor_m.begin())->second);
        // Sprawdzenie odczytow czujnika sily.
        check_force_condition(*the_sensor);
        }
    // Sprawdzenie, czy nie wykonano calej trajektorii.
    if(!is_interpose_list_element()){
        // Oproznienie listy.
        flush_interpose_list();
        // Przesuniecie sie na nastepna pozycje w liscie makrokrokow.
        next_pose_list_ptr();
        // Koniec ruchu.
        return false;
        } // end: if
    // Pobranie pozycji posredniej.
    get_interpose_list_element(interpose);
    // Stworzenie polecenia dla robota.
    create_command_for_pose(interpose);
    // Przesuniecie na nastepny element z listy pozycji posrednich.
    next_interpose_list_element();
    // Nie wykonano jeszcze calego ruchu.
    return true;
    } // end: next_step

/*********************  CHECK FORCE CONDITION **************************/
void trajectory_reproduce::check_force_condition(ecp_mp::sensor::force& the_sensor){
	lib::SENSOR_IMAGE si;
	// Pobranie odczytow z czujnika sily.
	the_sensor.get_reading(si);
    memcpy(last_force_sensor_reading, si.sensor_union.force.rez, MAX_SERVOS_NR*sizeof(double));
    // Sprawdzenie, czy nie wystapila za duza sila.
    for (int i=0; i<6; i++)
        if (fabs(last_force_sensor_reading[i]) > dangerous_force)
                throw ECP_error(lib::NON_FATAL_ERROR, DANGEROUS_FORCE_DETECTED);
    // Sila w porzadku.
    } // end: check_force_condition

/*********************** DANGEROUS FORCE HANDLER **************************/
void trajectory_reproduce::dangerous_force_handler(generator::ECP_error e){
    // Komunikat o bledzie wysylamy do SR.
    sr_ecp_msg.message (lib::NON_FATAL_ERROR, e.error_no);
    // Wiadomosc wysylana do UI.
    lib::ECP_message ecp_ui_msg;
    // Odswiezenie okna.
    ecp_ui_msg.hdr.type=0;
    // Polecenie ustawienia przyciskow.
    ecp_ui_msg.ecp_message = lib::TR_DANGEROUS_FORCE_DETECTED;
    // Przepisanie ostatniego odczytu czujnika sily.
   memcpy(ecp_ui_msg.R2S.force_sensor_reading, last_force_sensor_reading, MAX_SERVOS_NR*sizeof(double));
    // Wyslanie polecenia do UI.
#if !defined(USE_MESSIP_SRR)
		if (MsgSend(UI_fd, &ecp_ui_msg, sizeof(lib::ECP_message), NULL, 0) < 0){
#else
		int32_t answer;
		if (messip_send(UI_fd, 0, 0, &ecp_ui_msg, sizeof(lib::ECP_message), &answer, NULL, 0, MESSIP_NOTIMEOUT) < 0){
#endif
         sr_ecp_msg.message (lib::SYSTEM_ERROR, errno, "ECP: Send() to UI failed");
    }else
        sr_ecp_msg.message("Press TRY AGAIN to continue move.");
    } // end: dangerous_force_handler


/****************************** LOAD TRAJECTORY ******************************/
void trajectory_reproduce::load_trajectory(char* filename) {
	// Opis wspolrzednych: "MOTOR", "JOINT", ...
	char coordinate_type[80];
	// Rodzaj wspolrzednych
	lib::POSE_SPECIFICATION ps;
	double coordinates[6];                      // Wczytane wspolrzedne
	double motion_time;                         // Czas dojscia do wspolrzednych
	uint64_t number_of_poses;  // Liczba zapamietanych pozycji
	// uint64_t i, j;                         // Liczniki petli
	bool create_head = true;         // Tworzenie glowy czy wstawianie n-tego elementu
// printf("loadTrajectory: %s\n",filename);
try{
    // Otworzenie pliku do odczytu.
	std::ifstream from_file(filename);
    if (!from_file.good()){
        throw common::ECP_main_error(lib::NON_FATAL_ERROR, NON_EXISTENT_FILE);
        }
    // Wczytanie rodzaju wspolrzednych.
    if ( !(from_file >> coordinate_type) ) {
        throw common::ECP_main_error(lib::NON_FATAL_ERROR, NON_EXISTENT_FILE);
        }
    // Usuniecie spacji i tabulacji.
    unsigned int i = 0;
    unsigned int j = 0;
    while ( coordinate_type[i] == ' ' || coordinate_type[i] == '\t')
        i++;
    while ( coordinate_type[i] != ' '   && coordinate_type[i] != '\t' &&
     coordinate_type[i] != '\n'  && coordinate_type[i] != '\r' && coordinate_type[j] != '\0' ) {
        coordinate_type[j] = toupper(coordinate_type[i]);
        i++;
        j++;
        }
    coordinate_type[j] = '\0';
    // Sprawdzenie rodzaju wspolrzednych.
    if ( !strcmp(coordinate_type, "MOTOR") )
        ps = lib::MOTOR;
   else{
        throw common::ECP_main_error(lib::NON_FATAL_ERROR, NON_TRAJECTORY_FILE);
        }
    // Wczytanie liczby elementow.
    if ( !(from_file >> number_of_poses) ){
        throw common::ECP_main_error(lib::NON_FATAL_ERROR, READ_FILE_ERROR);
        }
    // Musi byc wiecej niz 0 elementow.
    if ( number_of_poses <1){
        throw common::ECP_main_error(lib::NON_FATAL_ERROR, NON_COMPATIBLE_LISTS);
        }
    // Usuniecie listy pozycji, o ile istnieje.
    flush_pose_list();
    // Dodanie kolejnych pozycji do listy.
    for ( i = 0; i < number_of_poses; i++){
        // Czas wykonywania ruchu.
        if (!(from_file >> motion_time)){
            // Zabezpieczenie przed danymi nienumerycznymi.
            throw common::ECP_main_error(lib::NON_FATAL_ERROR, READ_FILE_ERROR);
            }
        // Kolejne wspolrzedne.
        for ( j = 0; j < 6; j++) {
            if ( !(from_file >> coordinates[j]) ){
                // Zabezpieczenie przed danymi nienumerycznymi.
                throw common::ECP_main_error(lib::NON_FATAL_ERROR, READ_FILE_ERROR);
                }
            }
//     printf("Wczytano pozycje %i: czas %f | %f, %f, %f, %f, %f, %f\n",i, motion_time,
//         coordinates[0], coordinates[1], coordinates[2], coordinates[3], coordinates[4], coordinates[5]);
        // Dodanie elementu.
        if (create_head){
            // Stworzenie glowy listy.
            create_head = false;
            create_pose_list_head(ps, motion_time, coordinates);
        }else{
              // Wstawienie do listy nowej pozycji.
            insert_pose_list_element(ps,motion_time,coordinates);
            }
        } // end: for
    // Zamkniecie pliku.
    sr_ecp_msg.message("Trajectory readed properly.");
}catch (common::ECP_main_error e){
    // Wylapanie i oblsuga bledow.
    sr_ecp_msg.message (e.error_class, e.error_no);
}catch (...){
    // Wylapywanie niezdefiniowanych bledow.
    sr_ecp_msg.message (lib::NON_FATAL_ERROR, ECP_UNIDENTIFIED_ERROR);
    } // end: catch
} // end load_trajectory


/*************************  SET DANGEROUS FORCE *****************************/
void trajectory_reproduce::set_dangerous_force(void){
	dangerous_force = ecp_t.config.return_int_value("dangerous_force");
}


/*****************************  KONSTRUKTOR *********************************/
trajectory_reproduce::trajectory_reproduce (common::task::task& _ecp_task):
	 ecp_teach_in_generator (_ecp_task)
{
    // Ustawienie elementow list na NULL.
//    pose_list_ptr = NULL;
//    current_pose_ptr = NULL;
    interpose_list.clear();

    // Ustawienie niebezpiecznej sily.

    UI_fd = _ecp_task.UI_fd;
    set_dangerous_force();
    } // end: trajectory_reproduce_generator

/******************************  DESTRUKTOR **********************************/
trajectory_reproduce::~trajectory_reproduce (void) {
    // Usuniecie elementow z wszystkich list.
    flush_pose_list();
    flush_interpose_list();
    } // end: trajectory_reproduce_generator


/************* METODY ZWIAZANE Z LISTA POZYCJI  POSREDNICH *****************/
void trajectory_reproduce::flush_interpose_list ( void ) {
   interpose_list.clear();
    } // end: flush_interpose_list

void trajectory_reproduce::initiate_interpose_list(void) {
    interpose_list_iterator = interpose_list.begin();
    } // end: initiate_interpose_list

void trajectory_reproduce::next_interpose_list_element (void) {
    // Przejscie na nastepny element.
    interpose_list_iterator++;
    } // end: next_interpose_list_element

void trajectory_reproduce::get_interpose_list_element (common::ecp_taught_in_pose& tip){
    // Przepisanie danych ruchu.
    tip.arm_type = interpose_list_iterator->arm_type;
    tip.motion_time = interpose_list_iterator->motion_time;
    // Przepisanie polozenia z listy.
    memcpy(tip.coordinates, interpose_list_iterator->coordinates, MAX_SERVOS_NR*sizeof(double));
    } // end: get_interpose_list_element

bool trajectory_reproduce::is_interpose_list_element ( void ) {
    // Sprawdza czy element nie jest NULL.
    if (interpose_list_iterator != interpose_list.end())
        return true;
    else
        return false;
    } // end: is_interpose_list_element

void trajectory_reproduce::create_interpose_list_head (lib::POSE_SPECIFICATION ps, double motion_time, double coordinates[6]) {
    // Wstawienie glowy.
    	interpose_list.push_back(common::ecp_taught_in_pose(ps, motion_time, coordinates));
	interpose_list_iterator = interpose_list.begin();
    } // end: create_interpose_list_head

void trajectory_reproduce::insert_interpose_list_element (lib::POSE_SPECIFICATION ps, double motion_time, double coordinates[6]) {
    // Wlasciwe wstawienie elementu.
    	interpose_list.push_back(common::ecp_taught_in_pose(ps, motion_time, coordinates));
	interpose_list_iterator++;
    } // end: insert_interpose_list_element

/**********  KONIEC: METODY ZWIAZANE Z LISTA POZYCJI POSREDNICH ************/

}
} // namespace irp6ot
} // namespace ecp
} // namespace mrrocpp


