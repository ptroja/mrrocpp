///////////////////////////////////////////////////////////
//  ecp_vis_pb_eol_sac_irp6ot.h
//  Implementation of the Class ecp_vis_pb_eol_sac_irp6ot
//  Created on:      04-sie-2008 14:25:57
//  Original author: tkornuta
///////////////////////////////////////////////////////////

#if !defined(EA_B10EA6BB_2E03_4f87_8E50_B6482560D9BB__INCLUDED_)
#define EA_B10EA6BB_2E03_4f87_8E50_B6482560D9BB__INCLUDED_

#include "ecp/common/ecp_visual_servo.h"

class ecp_vis_pb_eol_sac_irp6ot : public ecp_visual_servo
{
//protected:
//sensor *vsp_force_irp6ot, *vsp_force_irp6p, *vsp_vis_sac;

public:
	sensor *vsp_force_irp6ot, *vsp_force_irp6p, *vsp_vis_sac;
	trajectory_description td;
  	int step_no;
  	int idle_step_counter; 
	double delta[6];
     
	/*!
	 * Entities:
	 */
	/*!
	 * ^{C}T_{G} -- goal pose with respect to the camera frame.
	 */
	Homog_matrix C_Tx_G;
	Homog_matrix C_Tx_E;
	Homog_matrix O_Tx_G;
	Homog_matrix O_Tx_Ep;
	Homog_matrix O_Tx_E;
				
	double C_r_G[3][6];
	double C_r_E[3][6];
	double O_r_G[3][6];
	double O_r_Ep[3][6];
	double O_r_E[3][6];
	double O_eps_E[3][6];
	double O_eps_EG[3][6];
	double O_eps_EG_norm;
				
	double O_r_Ep_d[3][6]; //roznica 1szego
	double O_r_Ep_d2[3][6]; //2giego stopnia
	
	/*!
	 * Operations:
	 */
	Homog_matrix O_Tx_C;
	double gain[6];
	
	double x2g;
	Homog_matrix G_Tx_G2;
	Homog_matrix G_Tx_S;

	ecp_vis_pb_eol_sac_irp6ot(ecp_task& _ecp_task, int step=0);
	virtual ~ecp_vis_pb_eol_sac_irp6ot();

	virtual void next_step_without_constraints();
	virtual void entertain_constraints();
	virtual bool first_step(void);

};
#endif // !defined(EA_B10EA6BB_2E03_4f87_8E50_B6482560D9BB__INCLUDED_)
