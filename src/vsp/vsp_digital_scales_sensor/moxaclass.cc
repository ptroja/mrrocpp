// -------------------------------------------------------------------------
// Plik:			MOXAClass.cc
// Opis:		metody klasy MOXA DIGITAL SCALE
// Autor:		tkornuta
// Data:		25.02.2005
// -------------------------------------------------------------------------

#include <sys/neutrino.h> // ThreadCtl
#include <sys/mman.h> // mmap
#include <hw/pci.h>
#include <hw/pci_devices.h>
#include <stdio.h>
#include <stdlib.h>
#include <termios.h> // do tcflush
#include <fcntl.h> // do flagi O_RDRW
#include <string.h>

#include <unistd.h> // do delay
#include <hw/inout.h>// do in*/out*

#include "common/com_buf.h"

#include "common/sensor.h"
#include "vsp/moxaclass.h"

/***************************** INCREMENT MARKER ******************************/
void MOXADigitalScale::increment_marker(int * marker){
    if (*marker < (CYCLIC_BUFFER_SIZE - 1))
        (*marker)++;
    else
        *marker = 0;
    }; // end: increment_marker

/***************************** EXTRACT MESSAGE *******************************/
void MOXADigitalScale::extract_message(void){
    message_length = 0;
    // Przepisanie wiadomosci z bufora cyklicznego.
    while(*(cyclic_buffer+read_marker) != '\r'){
        // Spawdzenie, czy nie zaszlo oproznienie bufora.
        if(read_marker == write_marker)
            throw sensor::sensor_error(FATAL_ERROR, CYCLIC_BUFFER_UNDERRUN);
        // Przepisanie znaku.
        *(message_buffer+(message_length++)) = *(cyclic_buffer+read_marker);
        // Przesuniecie znacznika odczytu na nastepna pozycje.
        increment_marker(&read_marker);
        };
    // Przesuniecie znacznika na nastepna wiadomosc.
    read_marker = write_marker;
    }; // end: extract_message

/******************************* KONSTRUKTOR *********************************/
MOXADigitalScale::MOXADigitalScale(short RS_number){
    // Przechoduje tryby odczytu/zapisu z portu RS-232.
    struct termios RS232_mode;
    // Znacznik zapisu znajduje sie na ostatniej zajetej pozycji z poprzedniej wiadomosci.
    write_marker = 0;
    // Znacznik odczytu znajduje sie na pierwszej zajetej pozycji z obecnej wiadomosci.
    read_marker = 0;
    // Przechowuje informacje o dlugosci wiadomosci.
    message_length = -1;
    // Dostep do sprzetu.
    ThreadCtl( _NTO_TCTL_IO, 0 );
    // Ustawienie dekryptora pliku w zaleznosci od numeru portu RS-232.
    switch(RS_number){
        case 1: RS_descriptor = open ("/dev/ser3", O_RDWR); break;
        case 2: RS_descriptor = open ("/dev/ser4", O_RDWR); break;
        case 3: RS_descriptor = open ("/dev/ser5", O_RDWR); break;
        case 4: RS_descriptor = open ("/dev/ser6", O_RDWR); break;
        case 5: RS_descriptor = open ("/dev/ser7", O_RDWR); break;
        case 6: RS_descriptor = open ("/dev/ser8", O_RDWR); break;
        case 7: RS_descriptor = open ("/dev/ser9", O_RDWR); break;
        case 8: RS_descriptor = open ("/dev/ser10", O_RDWR); break;
        }; // end: switch
    // Ustawienie parameterow transmisji.
    tcgetattr(RS_descriptor, &RS232_mode);
    cfsetispeed(&RS232_mode,600);
    cfsetospeed(&RS232_mode,600);
    RS232_mode.c_cflag = CREAD | CS7 | CSTOPB| PARENB;
    tcsetattr(RS_descriptor, TCSANOW, &RS232_mode);
    // Usuniecie znakow z ukladu UART.
    tcflush(RS_descriptor, TCIOFLUSH);
    }; // end: MOXADigitalScale

/******************************** DESTRUCTOR *********************************/
MOXADigitalScale::~MOXADigitalScale(void){
    // Zamkniecie deskryptora urzadzenia.
    close(RS_descriptor);
    };

/******************************* GET READING *********************************/
void MOXADigitalScale::get_reading(void){
    // Wylaczenie diody w OPTO-RS na 51 ms.
    tcdropline(RS_descriptor, 51);
    // Pelta odczytu znakow z ukladu UART.
    while(1){
        // Oczekiwanie na odczyt.
        char RS_read_buffer[20];
        int  RS_read_length = read (RS_descriptor, RS_read_buffer, sizeof(RS_read_buffer));
        // Przepisanie znakow do bufora cyklicznego.
        int i = 0;
        while(i < RS_read_length){
/*          // Wypisanie otrzymanego znaku.
            if(*(RS_read_buffer+i) != '\r')
                printf(" - %c\n",*(RS_read_buffer+i));
            else
                printf("<CR>\n");
*/
            // Przepisanie jednego znaku do bufora cyklicznego.
            *(cyclic_buffer+write_marker) = *(RS_read_buffer+i);
            // Przesuniecie znacznika zapisu na nastepna pozycje.
            increment_marker(&write_marker);
            // Sprawdzenie, czy nie przepelniono bufora.
            if(write_marker == read_marker)
                throw sensor::sensor_error(FATAL_ERROR, CYCLIC_BUFFER_OVERFLOW);
            // Przesuniecie sie na nastepny znak w bforze RS_read_buffer.
            i++;
            }; // end: while
        // Sprawdzenie, czy otrzymano cala wiadomosc.
        if(*(RS_read_buffer+RS_read_length-1) == '\r')
            break;
        }; // end: while(1)
    }; // end: get_reading

/*********************** TRANSFORM READING TO DOUBLE *************************/
double MOXADigitalScale::transform_reading_to_double(void){
    // Odczytanie wiadomosci z bufora cyklicznego.
    extract_message();
/*  // Wypisanie wiadomosci.
    printf("Received message (%i): ",message_length);
    for(int i=0; i < message_length; i++)
        printf("%c", *(message_buffer+i));
    printf("\n");
*/
    int i=0;
    // Odnalezienie znaku odczytu.
    while((*(message_buffer+i) != '+')&&(*(message_buffer+i) != '-')){
        i++;
        // Sprawdzenie, czy nie przekroczono wielkosci wiadomosci.
        if (i == message_length)
            throw sensor::sensor_error(FATAL_ERROR, CYCLIC_BUFFER_PARSE_ERROR);
        }; // end: while
    // Zapamietanie znaku.
    char sign = *(message_buffer+(i++));
    // Zmienna tymczasowa - odczytana cyfra.
    short tmp;
    // Odczyt z linialu.
    double reading =0;
    // Odczytanie liczby przed przecinkiem.
    while(*(message_buffer+i) != '.'){
        tmp = (*(message_buffer+i) - '0');
        reading = reading*10 + (short)tmp;
        i++;
        // Sprawdzenie, czy nie przekroczono wielkosci wiadomosci.
        if (i == message_length)
            throw sensor::sensor_error(FATAL_ERROR, CYCLIC_BUFFER_PARSE_ERROR);
        }; // end: while
    // Przesuniecie sie za przecinek.
    i++;
    // Mnoznik.
    double fraction = 0.1;
    // Odczytanie liczby po przecinku.
    while(i < message_length){
        tmp = (*(message_buffer+i) - '0');
        reading +=  ((double)tmp)*fraction;
        fraction *= 0.1;
        i++;
        };
    // Zmiana znaku odczytu.
    if( sign == '-')
        reading *= -1;
    //printf("odczyt = %f\n",reading);
    // Zwrocenie odczytu.
    return reading;
    }; // end: transform_reading_to_double
