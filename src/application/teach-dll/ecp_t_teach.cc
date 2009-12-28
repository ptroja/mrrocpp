// ------------------------------------------------------------------------
//
//                     EFFECTOR CONTROL PROCESS (lib::ECP) - main()
//
// Ostatnia modyfikacja: 2006
// ------------------------------------------------------------------------

#include <stdio.h>
#include <string.h>
#include <iostream>
#include <dlfcn.h>


#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"
#include "lib/srlib.h"
#include "ecp/irp6_on_track/ecp_r_irp6ot.h"
#include "ecp/irp6_postument/ecp_r_irp6p.h"
#include "ecp/irp6_mechatronika/ecp_r_irp6m.h"

#include "application/teach-dll/ecp_t_teach.h"
#include "application/teach-dll/ecp_g_teach.h"

#include "ecp/common/ecp_robot.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace task {

// KONSTRUKTORY
teach::teach(lib::configurator &_config) : task(_config)
{
    if (config.section_name == ECP_IRP6_ON_TRACK_SECTION)
    {
        ecp_m_robot = new irp6ot::robot (*this);
    }
    else if (config.section_name == ECP_IRP6_POSTUMENT_SECTION)
    {
        ecp_m_robot = new irp6p::robot (*this);
    }
    else if (config.section_name == ECP_IRP6_MECHATRONIKA_SECTION)
    {
        ecp_m_robot = new irp6m::robot (*this);
    }
    else {
    	fprintf(stderr, "unknown robot \"%s\" in teach task\n", config.section_name.c_str());
    	throw(ecp_robot::ECP_main_error(lib::FATAL_ERROR, 0));
    }

    tig = new generator::teach_tmp (*this);

    sr_ecp_msg->message("ECP loaded");
}


void teach::main_task_algorithm(void)
{

	 using std::cout;
	    using std::cerr;

	    cout << "C++ dlopen demo\n\n";

	    // open the library
	    cout << "Opening hello.so...\n";
	    void* handle = dlopen("./hello.so", RTLD_LAZY);

	    if (!handle) {
	        cerr << "Cannot open library: " << dlerror() << '\n';
	    //    return 1;
	    }

	    // load the symbol
	    cout << "Loading symbol hello...\n";
	    typedef void (*hello_t)();
	    hello_t hello = (hello_t) dlsym(handle, "hello");
	    if (!hello) {
	        cerr << "Cannot load symbol 'hello': " << dlerror() <<
	            '\n';
	        dlclose(handle);
	     //   return 1;
	    }

	    // use it to do the calculation
	    cout << "Calling hello...\n";
	    hello();

	    // close the library
	    cout << "Closing library...\n";
	    dlclose(handle);




	    flushall();

/*
	     // load the triangle library
	     void* gener = dlopen("./generator.so", RTLD_LAZY);
	     if (!gener) {
	         cerr << "Cannot load library: " << dlerror() << '\n';
	         return 1;
	     }

	     // load the symbols
	     create_t* create_gener = (create_t*) dlsym(gener, "create");
	     destroy_t* destroy_gener = (destroy_t*) dlsym(gener, "destroy");
	     if (!create_gener || !destroy_gener) {
	         cerr << "Cannot load symbols: " << dlerror() << '\n';
	         return 1;
	     }

	     // create an instance of the class
	     polygon* poly = create_gener();

	     // use the class
	     poly->set_side_length(7);
	         cout << "The area is: " << poly->area() << '\n';

	     // destroy the class
	     destroy_triangle(poly);

	     // unload the triangle library
	     dlclose(triangle);



*/




    switch (ecp_m_robot->robot_name)
    {
    case lib::ROBOT_IRP6_ON_TRACK:
        sr_ecp_msg->message("ECP teach irp6ot");
        break;
    case lib::ROBOT_IRP6_POSTUMENT:
        sr_ecp_msg->message("ECP teach irp6p");
        break;
    default:
        fprintf(stderr, "%s:%d unknown robot type\n", __FILE__, __LINE__);
    }

    if ( operator_reaction ("Teach in? ") )
    {
        tig->flush_pose_list(); // Usuniecie listy pozycji, o ile istnieje
        tig->teach (lib::ECP_MOTOR, "Teach-in the trajectory\n");
    }

    if ( operator_reaction ("Save trajectory? ") )
    {
        tig->save_file (lib::ECP_MOTOR);
    }

    if ( operator_reaction ("Load trajectory? ") )
    {
        tig->load_file_from_ui ();
    }

    // Aktualnie petla wykonuje sie jednokrotnie, gdyby MP przejal sterowanie
    // to petle mozna przerwac przez STOP lub przez polecenie lib::END_MOTION wydane
    // przez MP
    //  printf("w ecp for\n");
    tig->Move();
    // 	 printf("w ecp for za move\n");
    // Oczekiwanie na STOP
    ecp_termination_notice();
}

task* return_created_ecp_task (lib::configurator &_config)
{
	return new teach(_config);
}

} // namespace task
} // namespace common
} // namespace ecp
} // namespace mrrocpp

