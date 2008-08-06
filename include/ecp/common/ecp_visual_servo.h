/*!
 * \file ecp_visual_servo.h
 * \brief Abstract class responsible for visual servo. 
 * - class definition
 * \author tkornuta/mstaniak
 * \date 04.08.2008
 */


#if !defined(EA_8CD1135B_079F_4b66_82F9_362F9BC528CE__INCLUDED_)
#define EA_8CD1135B_079F_4b66_82F9_362F9BC528CE__INCLUDED_

#include "lib/mathtr.h"
#include "ecp/common/ecp_generator.h"

#if 0

// tutaj struktury
//...

/*!
 * \struct vis_constraints_t
 * \brief ... structure.
 * \author tkornuta/mstaniak
 */
struct vis_constraints_t
{
	/*!
	 * \union task_union_t
	 * \brief Union with task structures.
	 */
	union task_union_t
	{
		/*!
		 * \struct tk_testexample_struct_t
		 * \brief Structure used for test example purposes.
		 */
		struct tk_testexample_struct_t
		{
			/*!
			 * Character send between processor and panel.
			 */
			char character;
		} tk_testexample;		
	};
};



/*!
 * \struct vis_entities_t
 * \brief ... structure.
 * \author tkornuta/mstaniak
 */
struct vis_entities_t
{
	/*!
	 * \union vis_entities_union_t
	 * \brief Union with entities' structures.
	 */
	union vis_entities_union_t
	{	
		/*!
		 * \struct pb_eol_sac_struct_t
		 * \brief Structure used by PB-EOL-SAC.
		 */
		struct pb_eol_sac_struct_t
		{
			/*!
			 * ^{C}T_{G} -- goal pose with respect to the camera frame.
			 */

				Homog_matrix *C_Tx_G;
/*				Homog_matrix C_Tx_E;
				Homog_matrix O_Tx_G;
				Homog_matrix O_Tx_Ep;
				Homog_matrix O_Tx_E;
*/				
				double C_r_G[3][6];
				double C_r_E[3][6];
				double O_r_G[3][6];
				double O_r_Ep[3][6];
				double O_r_E[3][6];
				
				double O_eps_EG[3][6];
				double O_eps_EG__CSAC_norm;
				
				double O_r_Ep_d[3][6]; //roznica 1szego
				double O_r_Ep_d2[3][6]; //2giego stopnia
			
		} pb_eol_sac;
		
		vis_entities_union_t(){
		 pb_eol_sac.C_Tx_G = new Homog_matrix();
		};		
	};
		Homog_matrix C_Tx_E;
};

/*!
 * \struct vis_operations_t
 * \brief ... structure.
 * \author tkornuta/mstaniak
 */

struct vis_operations_t
{
	/*!
	 * \union vis_operations_union_t
	 * \brief Union with operations' structures.
	 */
	union vis_operations_union_t
	{
		/*!
		 * \struct pb_eol_sac_struct_t
		 * \brief Structure used by PB-EOL-SAC.
		 */
		struct pb_eol_sac_struct_t
		{
		
			//	Homog_matrix O_Tx_C;
			/*!
			 * ^{0}M^{*} -- gain.
			 */		
			double gain[6];
			
		} pb_eol_sac;		
	};
	//Homog_matrix G_Tx_G2;
	//Homog_matrix G_Tx_S;
};
#endif

/*!
 * \class ecp_visual_servo
 * \brief Abstract class responsible for visual servo.
 * \author tkornuta/mstaniak
 */
class ecp_visual_servo : public ecp_generator
{
private:
	/*!
	 *	sfbveiaolmv 
	 */
	int state;

public:
	/*!
	 * Constructor. 
	 */
	 
	double measure_border_u[6];
	double measure_border_d[6];
	double d_u_max[6];
	double d2_u_max[6];
	 
	ecp_visual_servo(ecp_task& _ecp_task, int step=0);
	  
	virtual ~ecp_visual_servo();

	virtual void retrieve_parameters();
	virtual bool next_step(void);
	virtual void next_step_without_constraints() =0;
	virtual void entertain_constraints() =0;
	void set_constraints();
	void get_constraints();
	void set_entities();
	void get_entities();
	void set_opartions();
	void get_operations();
	//virtual void generate_config_labels() =0;


};
#endif // !defined(EA_8CD1135B_079F_4b66_82F9_362F9BC528CE__INCLUDED_)
