// -------------------------------------------------------------------------
//                            const.h
// Stale wykorzystywane w proce sie UI
// 
// Ostatnia modyfikacja: 16.04.98
// -------------------------------------------------------------------------

#ifndef __CONST_H
#define __CONST_H

// ------------------------------------------------------------------------- 
// Zdefiniowane nowe metki (tags) dla elementow w celu ich      
// latwiejszej  selekcji w roznych stanach systemu.              
// Nowe metki maja nastepujacy format:                           
// 6 znakow nazwy ewentualnie dopelnione znakiem '0' oraz 8 znakow kodu    
// opisujacego stan systemu, w ktorym element ma byc dostepny    
// dla kolejnego stanu kolejne miejsce, np;                      
// #define EXAMPLE_TAG "examp011011010"                          
//                      12345612345678                           
// aktualnie w system moze znajdowac sie w 8 stanach, a element  
// definiowany za pomoca EXAMPLE_TAG moze byc dostepny w stanach:
// 1,2,4,5,7                                                     

                    // 12345612345678
#define  LEDP1_TAG    "ledp1010000000" // zaladowanie EDP
#define  UEDP1_TAG    "uedp1001000000" // usuniecie EDP
#define  LMP_TAG      "lmp00001000000" // zaladowanie MP
#define  UMP_TAG      "ump00000100000" // usuniecie MP
#define  MANUAL_TAG   "manual01101000" // ruchy reczne
#define  ECPTEACH_TAG "mteach00100000" // uczenie i zapamietywanie pozycji
#define  START_TAG    "start000100000" // uruchomienie MP
#define  STOP_TAG     "stop0000011000" // zatrzymanie MP
#define  PAUSE_TAG    "pause000010000" // zawieszenie MP przez proxy
#define  PAUSEH_TAG   "pauseH00010000" // zawieszenie MP sygnalem
#define  RESUME_TAG   "resume00000100" // wznowienie MP przez proxy
#define  RESUMEH_TAG  "resumH00000010"  // odwieszenie MP sygnalem
#define  HELP_TAG     "help0011111111" // pomoc
#define  QUIT_TAG     "quit0011111000" // koniec
#define  CONF_TAG     "conf0011111111" // konfigurowanie UI
#define  INFO_TAG     "info0011111111" // informacja o sterowniku

// Czas przeterminowania oczekiwania na wiadomosc od procesu MP oraz 
// okres co jaki czas sprawdzane jest nadejscie przesylki
#define MP_TIMEOUT 20000
#define MP_TIMEOUT_SLICE 1000

// Wzorce metek elementow aktywnych w poszczegolnych stanach
                                       // 12345612345678
#define INITIAL_STATE_PATTERN            "??????1*"
#define MANUAL_OPERATIONS_PATTERN        "???????1*"
#define MP_LOADED_MANUAL_PATTERN         "????????1*"
#define MP_RUNNING_PATTERN               "?????????1*"
#define ECP_TEACHING_PATTERN             "??????????1*"
#define MP_PAUSED_PATTERN                "???????????1*"
#define MP_PAUSED_H_PATTERN              "????????????1*"

// stale okreslajace opcje elemetu
#define ACTIV  "-d-L"  // d - dim, L - lock
#define PASSIV "dL"


// stale okreslajace komunikaty wyswietlane w 'bottom bar' w zaleznosci
// od stanu systemu
#define SR_TEXT        "SR loaded "
#define LEDP1_TEXT      "SR, EDP loaded"
#define LMP_LEDP1_TEXT  "SR, EDP, MP loaded"

// Stale opisujace wskazany klawisz
#define LEDP1_DSCR "Load Effector Driver Process"
#define UEDP1_DSCR "Unload Effector Driver Process"
#define LMP_DSCR "Load Master Process"
#define UMP_DSCR "Unload Master Process"
#define MANUAL_DSCR "Open Manual Operations Menu"
#define START_DSCR "Run Master Process"
#define PAUSE_DSCR "Pause Master Process (by trigger proxy to Master Process)"  
#define PAUSEH_DSCR "Pause Master Process (by raise signal to Master Process)" 
#define RESUME_DSCR "Resume Master Process (by trigger proxy to Master Process)"
#define RESUMEH_DSCR "Resume Master Process (by raise signal to Master Process)"
#define STOP_DSCR "Stop Master Process"
#define HELP_DSCR "Help"
#define QUIT_DSCR "Exit to QNX Shell"
#define CONF_DSCR "Robot Controller Configuration"
#define INFO_DSCR "Robot Controller Diagram" 

#define SYNCHRO_MSG "Robot synchronisation - please wait"
#define MOVING_MSG  "Robot moving - please wait"
#define LOADING_MSG "Effector Driver Process loading - please wait"

// Tytul okna wyswietlajacego komunikaty o bledach
#define UI_ERROR_TITLE "User Interface Error!!!"

   // Stany procesu UI
#define INITIAL_STATE          90 // zaladowano tylko SR
#define MANUAL_OPERATIONS     100 // zaladowano EDP1
#define MP_LOADED_MANUAL      110 // zaladowano MP
#define ECP_TEACHING          120 // uczenie robota
#define MP_RUNNING            130 // dziala MP
#define MP_PAUSED             140 // MP wstrzymany przez proxy
#define MP_PAUSED_H           150 // MP wstrzymany sygnalem



#endif // __CONST_H 

