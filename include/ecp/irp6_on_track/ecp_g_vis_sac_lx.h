// -------------------------------------------------------------------------
//                            ecp.h dla QNX6
// Deklaracje struktur danych i metod dla procesow ECP z wykrozsytsaniem sily
// 
// -------------------------------------------------------------------------

#if !defined(_ECP_GEN_VIS_SAC_LX_H)
#define _ECP_GEN_VIS_SAC_LX_H


#include "lib/mathtr.h"
#include "common/impconst.h"
#include "common/com_buf.h"

#include "ecp/common/ecp_generator.h"

class ecp_vis_sac_lx_generator : public ecp_generator {
protected:
sensor *vsp_force_irp6ot, *vsp_force_irp6p, *vsp_vis_sac;

public:	
  trajectory_description td;
/*  
int step_no;
  
  double delta[6];
  double frame1[4][4];
  double pose[6][5];
  double pose_v[6][5];
  double pose_a[6][5];
  double pose_d[6][5];
  double pose_d2[6][5];
  bool the_first;
  */
  	int step_no;
  	int idle_step_counter; 
	double delta[6];
     
     frame_tab my_goal_frame_m;
     
	Homog_matrix C_Tx_G;
	Homog_matrix C_Tx_E;
	Homog_matrix O_Tx_E;
	Homog_matrix O_Tx_Ep;
	Homog_matrix O_Tx_G;
	Homog_matrix G_Tx_G2;
	
	Homog_matrix G_Tx_S;
	
	Homog_matrix O_Tx_C;
	Homog_matrix O_Tx_E__C;
	Homog_matrix O_Tx_G__C;

	Homog_matrix E_Tx_G;
	Homog_matrix E_Tx_Ep;
	
	Homog_matrix E_Tx_G__O;
	
	//EIH
	Homog_matrix O_Tx_G__C2;
	Homog_matrix E_Tx_C2;
	Homog_matrix C2_Tx_G;

	Homog_matrix O_Tx_EE;	
	double O_r_G__C2[3][6];
	
	
	double O_r_E[3][6];
	double O_r_Ep[3][6];
	double O_r_Ep_d[3][6]; //roznica 1szego
	double O_r_Ep_d2[3][6]; //2giego stopnia
	
	double O_r_G[3][6];
	double O_rcom_G[3][6];
	double E_r_G[3][6];
	double E_r_Ep[3][6];
	
	double C_r_G[3][6];
	
	double O_reul_Ep[3][6];
	
	double O_eps_EG[3][6];
	double E_eps_EG[3][6]; //E_r_G; - prawdopodobnie to samo
  	
  	double measure_border_u[6];//={1.090, 0.150, 0.305, 0.606, 1.57, 3.12}; //Zmienic ogranicz Z
	double measure_border_d[6];//={0.82, -0.150, 0.155, -0.606, 1.267, 2.5}; // gamma 1.8


	double d_u_max[6];//={0.3, 0.3, 0.010, 0.3, 0.3, 0.3}; //td.internode_step_no = 80; //jeszcze nie w [m]
	double d2_u_max[6];//={0.1, 0.1, 0.01, 0.05, 0.05, 0.05}; //tylko ustalone dla Y

	double gain[6];
	
	double force_inertia_;
	double torque_inertia_;
	double force_reciprocal_damping_;
	double torque_reciprocal_damping_;

	double x2g;//=-0.07; //x nibytoola
	
	double x2g_begin;

int vis_phase;// = 0;
int steps2switch;//=0;
  	
  // konstruktor
	ecp_vis_sac_lx_generator(ecp_task& _ecp_task, int step=0);  

  virtual bool first_step ();
      // generuje pierwszy krok ruchu -
      // pierwszy krok czsto rni sie od pozostaych, 
      // np. do jego generacji nie wykorzystuje sie czujnikw
      // (zadanie realizowane przez klas konkretn)
  virtual bool next_step ();
     // generuje kady nastepny krok ruchu
     // (zadanie realizowane przez klas konkretn)
}; // end: class irp6ot_calibration_generator
// --------------------------------------------------------------------------

#endif
