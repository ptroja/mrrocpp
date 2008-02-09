#include <stdio.h>
#include <unistd.h>

#include "common/typedefs.h"
#include "common/impconst.h"
#include "common/com_buf.h"
#include "lib/srlib.h"
#include "mp/mp.h"
#include "mp/mp_t_c.h"
#include "mp/mp_common_generators.h"

mp_task* return_created_mp_task (void)
{
	return new mp_task_c();
}

// methods for mp template to redefine in concrete class
void mp_task_c::task_initialization(void) 
{
	//usleep(1000*100);
	sr_ecp_msg->message("MP c loaded");
};
 
void mp_task_c::main_task_algorithm(void)
{
   mp_empty_generator empty_gen (*this); // "Pusty" generator
   empty_gen.robot_m = robot_m;
   
      // Oczekiwanie na zlecenie START od UI  
      wait_for_start ();
      // Wyslanie START do wszystkich ECP 
      start_all (robot_m);

      for (;;) {  // Wewnetrzna petla    
        // Zlecenie wykonania kolejnego makrokroku
// 	   printf("po start all robot_list_head: %d, slhead: %d \n", robot_list_head, slhead);
        if ( Move (empty_gen) )
          break;

       	 // Oczekiwanie na STOP od UI
         //    printf("w mp przed wait for stop\n");
        wait_for_stop (MP_THROW); // by Y - wlaczony tryb
  
        // Wyslanie STOP do wszystkich ECP po zakonczeniu programu uzytkownika
        terminate_all (robot_m);
        break; 
      } // koniec: for(;;) - wewnetrzna petla
};
