/*!
 * \file ecp_visual_servo.h
 * \brief Abstract class responsible for visual servo. 
 * - class definition
 * \author tkornuta/mstaniak
 * \date 04.08.2008
 */


#if !defined(EA_8CD1135B_079F_4b66_82F9_362F9BC528CE__INCLUDED_)
#define EA_8CD1135B_079F_4b66_82F9_362F9BC528CE__INCLUDED_

#include "ecp/common/ecp_generator.h"

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
	virtual void generate_config_labels() =0;


};
#endif // !defined(EA_8CD1135B_079F_4b66_82F9_362F9BC528CE__INCLUDED_)
