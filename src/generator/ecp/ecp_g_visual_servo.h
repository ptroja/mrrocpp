/*!
 * \file ecp_visual_servo.h
 * \brief Abstract class as a pattern for implementing any visual servo.
 * - class declaration
 * \author tkornuta/mstaniak
 * \date 04.08.2008
 */


#if !defined(EA_8CD1135B_079F_4b66_82F9_362F9BC528CE__INCLUDED_)
#define EA_8CD1135B_079F_4b66_82F9_362F9BC528CE__INCLUDED_

#include "sys/time.h"
#include "base/lib/mrmath/mrmath.h"
#include "base/ecp/ecp_generator.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

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

				lib::Homog_matrix *C_Tx_G;
/*				lib::Homog_matrix C_Tx_E;
				lib::Homog_matrix O_Tx_G;
				lib::Homog_matrix O_Tx_Ep;
				lib::Homog_matrix O_Tx_E;
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
		 pb_eol_sac.C_Tx_G = new lib::Homog_matrix();
		};
	};
		lib::Homog_matrix C_Tx_E;
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

			//	lib::Homog_matrix O_Tx_C;
			/*!
			 * ^{0}M^{*} -- gain.
			 */
			double gain[6];

		} pb_eol_sac;
	};
	//lib::Homog_matrix G_Tx_G2;
	//lib::Homog_matrix G_Tx_S;
};
#endif



/*!
 * \class ecp_visual_servo
 * \brief Abstract class responsible for visual servo.
 * \author tkornuta/mstaniak
 */
class ecp_visual_servo : public common::generator::generator
{
private:

	int state;

public:

#if 1
/*!
	 * Entities:
	 */
	/*!
	 * ^{C}T_{G} -- goal pose with respect to the camera frame.
	 */
	lib::Homog_matrix C_Tx_G;
	/*!
	 * ^{C}T_{E} -- end-effector pose with respect to the camera frame.
	 */
	lib::Homog_matrix C_Tx_E;
	/*!
	 * ^{0}T_{G} -- goal pose with respect to the global frame.
	 */
	lib::Homog_matrix O_Tx_G;
	/*!
	 * ^{C}T_{E'} -- end-effector next pose with respect to the global frame.
	 */
	lib::Homog_matrix O_Tx_Ep;
	/*!
	 * ^{0}T_{E} -- goal pose with respect to the camera frame.
	 */
	lib::Homog_matrix O_Tx_E;
	/*!
	 * ^{C}r_{G} -- goal pose with respect to the camera frame (AA).
	 */
	double C_r_G[3][6];
	/*!
	 * ^{C}r_{E} -- goal pose with respect to the camera frame (AA).
	 */
	double C_r_E[3][6];
	/*!
	 * ^{0}r_{G} -- end-effector pose with respect to the global frame (AA).
	 */
	double O_r_G[3][6];
	/*!
	 * ^{0}r_{E'} -- end-effector next pose with respect to the global frame (AA).
	 */
	double O_r_Ep[3][6];
	/*!
	 * ^{0}r_{E} -- end-effector pose with respect to the camera frame (AA).
	 */
	double O_r_E[3][6];
	/*!
	 * ^{0}\varepsilon_{E} -- goal pose with respect to the camera frame (AA).
	 */
	double O_eps_E[3][6];
	/*!
	 * ^{0}\varepsilon_{EG} -- goal error with respect to the global frame (AA).
	 */
	double O_eps_EG[3][6];
	/*!
	 * \left|| ^{0}\varepsilon_{E} \right|| -- Euclid norm of goal error with respect to the global frame (AA).
	 */
	double O_eps_EG_norm;
	/*!
	 * roznica 1szego
	 */
	double O_r_Ep_d[3][6];
	/*!
	 * 2giego stopnia
	 */
	double O_r_Ep_d2[3][6];
	/*!
		 * ^{0}r_{E1} -- end-effector pose with respect to the camera frame (AA) in the fisrt step.
	*/
	double O_r_E1[6];

	/*!
	 * Operations:
	 */
	 /*!
	 * ^{0}r_{C} -- camera pose with respect to the camera frame.
	 */
	lib::Homog_matrix O_Tx_C;
	/*!
	 * M -- gains.
	 */
	double gain[6];

	double x2g;
	lib::Homog_matrix G_Tx_G2;
	lib::Homog_matrix G_Tx_S;
#endif

	/*!
	* max control value
	*/
	double measure_border_u[6];
	/*!
	* min control value
	*/
	double measure_border_d[6];
	/*!
	* max velocity
	*/
	double d_u_max[6];
	/*!
	* max acceleration
	*/
	double d2_u_max[6];

	struct timeval acctime;

	/*!
	 * Constructor.
	 */
	ecp_visual_servo(common::task::task& _ecp_task, int step=0);
	/*!
	 * Destructor.
	 */
	virtual ~ecp_visual_servo();

	virtual void retrieve_parameters();
	/*!
	 * The next step.
	 */
	virtual bool next_step(void);
	/*!
	* Method calcualting ^{0}r_{E'}
	*/
	virtual bool next_step_without_constraints() =0;
	/*!
	* Method aplying contrains -- AV()
	*/
	virtual void limit_step();
	void set_constraints();
	void get_constraints();
	void set_entities();
	void get_entities();
	void set_opartions();
	void get_operations();
	//virtual void generate_config_labels() =0;


};

} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif // !defined(EA_8CD1135B_079F_4b66_82F9_362F9BC528CE__INCLUDED_)
