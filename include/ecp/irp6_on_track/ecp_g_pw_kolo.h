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

namespace mrrocpp {
namespace ecp {
namespace irp6ot {

#define PI 3.14159265
#define DEG 0.0174444 //Jeden stopien wyrazony w radianach


class ecp_g_pw_kolo : public common::ecp_generator
{
    trajectory_description td;
    double next_position[8];  
    int step_no;
    double d_rad; //Przyrost k�ta w radianach.
    double prev_rad; //Wcz�niejszy k�t(koncowka, srodek kola)
    double r; //Promien rysowanego okregu.
    double y0; //�rodek ko�a - wsp y.
    double z0; //�rodek ko�a - wsp z.
    double y;	
    double z;
public:
    ecp_g_pw_kolo (common::task::ecp_task& _ecp_task);
    virtual bool first_step();
    virtual bool next_step();
};

} // namespace irp6ot
} // namespace ecp
} // namespace mrrocpp

#endif



