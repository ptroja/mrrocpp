// -------------------------------------------------------------------------
//						   Testowe kolo.
// Deklaracje generatora testowego rysujacego koncowka kolo. Koncowka porusz
// sie zgodnie z ruchem wskazowek zegara.
// -------------------------------------------------------------------------

#if !defined(_ECP_G_PW_KOLO_H)
# define _ECP_G_PW_KOLO_H

#include <string.h>
#include <math.h>

#include "ecp/common/ecp_generator.h"

class ecp_g_pw_kolo : public ecp_generator
{
    trajectory_description td;
    double next_position[8];  
    int step_no;
    double delta_y; //Przyrost zmiennej y.
    double r; //Promien rysowanego okregu.
    double y0; //¦rodek ko³a - wsp y.
    double z0; //¦rodek ko³a - wsp z.
    double y;	
    double z;
public:
    ecp_g_pw_kolo (ecp_task& _ecp_task);
    virtual bool first_step();
    virtual bool next_step();
};

#endif



