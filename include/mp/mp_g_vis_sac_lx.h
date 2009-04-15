#if !defined(__MP_GEN_VIS_SAC_LX_H)
#define __MP_GEN_VIS_SAC_LX_H

#include "lib/mathtr.h"

namespace mrrocpp {
namespace mp {
namespace generator {

// --------------------------------------------------------------------------
// Generator trajektorii dla zadan z wodzeniem za nos w tff ze zmiana orientacji

class vis_sac_lx : public generator 
{
protected:
  int idle_step_counter; // Licznik jalowych krokow sterowania (bez wykonywania ruchu)

  
  common::robot *irp6ot, *irp6p;
  lib::sensor *vsp_force_irp6ot, *vsp_force_irp6p, *vsp_vis_sac;
    
    // do konfiguracji pracy generatora
    unsigned short irp6ot_con, irp6p_con;
 

     lib::trajectory_description td;   
 
public:
	int step_no;
	double delta[6];
     
     lib::frame_tab my_goal_frame;
     
	lib::Homog_matrix C_Tx_G;
	lib::Homog_matrix C_Tx_E;
	lib::Homog_matrix O_Tx_E;
	lib::Homog_matrix O_Tx_Ep;
	lib::Homog_matrix O_Tx_G;
	lib::Homog_matrix G_Tx_G2;
	
	lib::Homog_matrix G_Tx_S;
	
	lib::Homog_matrix O_Tx_C;
	lib::Homog_matrix O_Tx_E__C;
	lib::Homog_matrix O_Tx_G__C;

	lib::Homog_matrix E_Tx_G;
	lib::Homog_matrix E_Tx_Ep;
	
	lib::Homog_matrix E_Tx_G__O;
	
	
	double O_r_E[3][6];
	double O_r_Ep[3][6];
	double O_r_Ep_d[3][6]; //roznica 1szego
	double O_r_Ep_d2[3][6]; //2giego stopnia
	
	double O_r_G[3][6];
	double E_r_G[3][6];
	double E_r_Ep[3][6];
	
	double O_reul_Ep[3][6];
	
	double O_eps_EG[3][6];
	double E_eps_EG[3][6]; //E_r_G; - prawdopodobnie to samo
	
    // konstruktor
    vis_sac_lx(task::base& _mp_task, int step=0);  
	
	virtual bool first_step ();    
	virtual bool next_step ();    

}; // end : class nose_run_force
} // namespace common
} // namespace mp
} // namespace mrrocpp
#endif
