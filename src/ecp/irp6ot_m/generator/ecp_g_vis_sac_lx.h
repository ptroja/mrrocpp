// -------------------------------------------------------------------------//                            ecp.h dla QNX6
// Deklaracje struktur danych i metod dla procesow ECP z wykrozsytsaniem sily
//
// -------------------------------------------------------------------------

#if !defined(_ECP_GEN_VIS_SAC_LX_H)
#define _ECP_GEN_VIS_SAC_LX_H


#include "lib/mrmath/mrmath.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "ecp/common/generator/ecp_generator.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot_m {
namespace generator {

class vis_sac_lx : public common::generator::generator {
protected:
lib::sensor *vsp_force_irp6ot, *vsp_force_irp6p, *vsp_vis_sac;


public:
  lib::trajectory_description td;
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

     lib::Homog_matrix my_goal_frame;

	lib::Homog_matrix C_Tx_G;
	lib::Homog_matrix CSAC_Tx_G;
	lib::Homog_matrix C_Tx_E;
	lib::Homog_matrix O_Tx_E;
	lib::Homog_matrix O_Tx_Ep;
	lib::Homog_matrix O_Tx_G;
	lib::Homog_matrix O_Tx_G__CSAC;
	lib::Homog_matrix G_Tx_G2;

	lib::Homog_matrix G_Tx_S;

	lib::Homog_matrix O_Tx_C;
	lib::Homog_matrix O_Tx_CSAC;
	lib::Homog_matrix O_Tx_E__C;
	lib::Homog_matrix O_Tx_G__C;

	lib::Homog_matrix E_Tx_G;
	lib::Homog_matrix E_Tx_Ep;

	lib::Homog_matrix E_Tx_G__O;

	//EIH
	lib::Homog_matrix O_Tx_G__C2;
	lib::Homog_matrix O_Tx_G__CEIH;
	lib::Homog_matrix E_Tx_C2;
	lib::Homog_matrix C2_Tx_G;
	lib::Homog_matrix O_Tx_G__fEIH;
	lib::Homog_matrix CEIH_Tx_G;
	lib::Homog_matrix CEIH_Tx_G__f;

	lib::Homog_matrix O_Tx_EE;

	lib::Homog_matrix O_Tx_G__D;
	lib::Homog_matrix G_Tx_D;

		lib::Homog_matrix O_Tx_G__BLOCK;

	 lib::Ft_v_vector O_rf_G__CEIH;
	lib::V_tr Jack;//O_Tx_G__CEIH,lib::Ft_v_tr::FT);
	//	lib::Ft_v_tr ft_tr_inv_tool_matrix (!current_tool, lib::Ft_v_tr::FT);

	int CSAC_Tx_G_firstvalid;
	int CEIH_Tx_G_firstvalid;

	double O_r_G__C2[3][6];


	double O_r_E[3][6];
	double O_r_Ep[3][6];
	double O_r_Ep_d[3][6]; //roznica 1szego
	double O_r_Ep_d2[3][6]; //2giego stopnia

	double O_r_G[3][6];
	double O_rcom_G[3][6];
	double O_r_G__CSAC[3][6];
	double O_r_G__CEIH[3][6];
	double O_r_G__fEIH[3][6];
	double O_r_G__D[3][6];

	double E_r_G[3][6];
	double E_r_Ep[3][6];

	double C_r_G[3][6];
	double CEIH_r_G[3][6];
	double CEIH_r_G__f[3][6];

	double CEIH_rpy_G[3][6];

	double O_reul_Ep[3][6];

	double O_eps_EG[3][6];
	double E_eps_EG[3][6]; //E_r_G; - prawdopodobnie to samo

	double O_eps_EG__CSAC[3][6];
	double O_eps_EG__CEIH[3][6];
	double O_eps_EG__fEIH[3][6];
	double O_eps_EG__D[3][6];

	double O_eps_E__CSAC[3][6];
	double O_eps_E__CEIH[3][6];
	double O_eps_E__fEIH[3][6];
	double O_eps_E__D[3][6];

  	double fEIH_G[8];

	double O_eps_EG__CSAC_norm;
	double O_eps_EG__CEIH_norm;
	double O_eps_EG__fEIH_norm;



  	double measure_border_u[6];//={1.090, 0.150, 0.305, 0.606, 1.57, 3.12}; //Zmienic ogranicz Z
	double measure_border_d[6];//={0.82, -0.150, 0.155, -0.606, 1.267, 2.5}; // gamma 1.8


	double d_u_max[6];//={0.3, 0.3, 0.010, 0.3, 0.3, 0.3}; //td.internode_step_no = 80; //jeszcze nie w [m]
	double d2_u_max[6];//={0.1, 0.1, 0.01, 0.05, 0.05, 0.05}; //tylko ustalone dla Y

	double gain[6];

	double O_gain__SAC[6];
	double C_gain__SAC[6];
	double f_gain__SAC[6];
	double C_gain__EIH[6];
	double f_gain__EIH[6];


	double O_weight__SAC;
	double C_weight__SAC;
	double f_weight__SAC;
	double C_weight__EIH;
	double f_weight__EIH;

	double force_inertia_;
	double torque_inertia_;
	double force_reciprocal_damping_;
	double torque_reciprocal_damping_;

	double x2g;//=-0.07; //x nibytoola

	double x2g_begin;

	double x2d;

int vis_phase;// = 0;
int steps2switch;//=0;

	int phaseCEIH;
	int phaseD;

  // konstruktor
	vis_sac_lx(common::task::task& _ecp_task, int step=0);

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

}
} // namespace irp6ot
} // namespace ecp
} // namespace mrrocpp

#endif
