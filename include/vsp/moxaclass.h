// -------------------------------------------------------------------------
// Plik:    MOXAClass.h
// Opis:    deklaracja klasy MOXA DIGITAL SCALE
// Autor:   tkornuta
// Data:    26.01.2005
// -------------------------------------------------------------------------

// Wielkosc buffora wiadomosci.
#define MESSAGE_BUFFER_SIZE 50

// Wielkosc bufora cyklicznego.
#define CYCLIC_BUFFER_SIZE 100

// ####################################################################
// #############    KLASA do odczytywania z linialow    ###############
// ####################################################################

class MOXADigitalScale{
private:
    // Znacznik zapisu znajduje sie na pierwszej wolnej pozycji, na ktorej mozna zapisac nastepny znak.
    int write_marker;
    // Znacznik odczytu znajduje sie na pierwszym znaku nowej wiadomosci.
    int read_marker;
    // Bufor cykliczny tymczasowo przechowuje znaki otrzymane z ukladu UART.
    char cyclic_buffer[CYCLIC_BUFFER_SIZE];
    // Deskryptor linialu.
    int RS_descriptor;
    // Odczytana wiadomosc, odczytana z bufora cyklicznego.
    char message_buffer[MESSAGE_BUFFER_SIZE];
    // Informacja o dlugosci wiadomosci.
    int message_length;
    // Przesuniecie sie znacznika na nastepne pole w buforze cyklicznym.
    void increment_marker(int * marker);
    // Odczytanie calej wiadomosci z bufora cyklicznego.
    void extract_message(void);
public:
    // Konstruktor.
    MOXADigitalScale(short RS_number);
    // Destruktor.
    ~MOXADigitalScale(void);
    // Pobranie odczytu.
    void get_reading(void);
    // // Przetworzenie odczytu do postaci zmiennoprzecinkowej.
    double transform_reading_to_double(void);
    }; // end: MOXADigitalScale
