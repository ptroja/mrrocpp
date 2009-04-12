// -------------------------------------------------------------------------
//                             ecp.cc
//             Effector Control Process (ECP) - methods
// Funkcje do tworzenia procesow ECP
//
//
// Ostatnia modyfikacja: 24.04.2006
// autor: tstempko
// -------------------------------------------------------------------------

#include <fstream>

#include "common/typedefs.h"
#include "common/impconst.h"
#include "common/com_buf.h"

#include "lib/srlib.h"

//#include "lib/matrix.h"
#include "lib/nurbs.h"
#include "lib/nurbs_tdes.h"
#include "ecp/irp6_on_track/ecp_local.h"
#include "ecp/irp6_on_track/ecp_g_nurbs.h"


namespace mrrocpp {
namespace ecp {
namespace irp6ot {

const size_t Dim=6;

using namespace NurbsLib;


template< POSE_SPECIFICATION arm_type, size_t D >
class Irp6ot_Point_nD : public Point_nD< D > {;};


using namespace std;


ostream& operator<<(ostream& s, const valarray<double>& v) {
	for (size_t  i = 0; i<v.size(); i++) 
		s << v[i] <<"\t"; 
	return s;
} 



//####################################################################################################
// Generator nurbs
//####################################################################################################

//---------------------------------  KONSTRUKTOR  ----------------------------------------------

irp6ot_nurbs_generator::irp6ot_nurbs_generator (common::task::ecp_task& _ecp_task,
	 const nurbs_tdes &ntdes, int mp_communication_mode_arg)
: ecp_generator (_ecp_task) 
{
	mp_communication_mode_=mp_communication_mode_arg;
	ntdes_ptr_ = &ntdes;
};

//----------------------------------------------------------------------------------------------
//---------------------------------    metoda	first_step -------------------------------------
//----------------------------------------------------------------------------------------------

bool irp6ot_nurbs_generator::first_step (  )
{

		EDP_data_next_ptr_=0;
//		cout<<"firststep(): B4 dynamic_cast \n"<<flush;
//		if (ntdes_ptr_->ncptr==NULL) cerr<<"Blad: ntdes.ncptr==NULL\n";
		if (dynamic_cast< NurbsCurve<Irp6ot_Point_nD< MOTOR, Dim > >* >(ntdes_ptr_->ncptr) ) {
			EDP_data_next_ptr_=&the_robot->EDP_data.next_motor_arm_coordinates[0];
			EDP_data_current_ptr_=&the_robot->EDP_data.current_motor_arm_coordinates[0];
//			cout<<"MOTOR\n";
			atype_=MOTOR; }
		else if (dynamic_cast< NurbsCurve<Irp6ot_Point_nD< JOINT, Dim > >* >(ntdes_ptr_->ncptr) ) {
			EDP_data_next_ptr_=&the_robot->EDP_data.next_joint_arm_coordinates[0];
			EDP_data_current_ptr_=&the_robot->EDP_data.current_joint_arm_coordinates[0];
//			cout<<"JOINT\n";
			atype_=JOINT; }
		else if (dynamic_cast< NurbsCurve<Irp6ot_Point_nD< XYZ_EULER_ZYZ, Dim > >* >(ntdes_ptr_->ncptr) ) {
			EDP_data_next_ptr_=&the_robot->EDP_data.next_XYZ_ZYZ_arm_coordinates[0];
			EDP_data_current_ptr_=&the_robot->EDP_data.current_XYZ_ZYZ_arm_coordinates[0];
//			cout<<"XYZ_EULER_ZYZ\n";
			atype_=XYZ_EULER_ZYZ; }
		else if (dynamic_cast< NurbsCurve<Irp6ot_Point_nD< XYZ_ANGLE_AXIS, Dim > >* >(ntdes_ptr_->ncptr) ) {
			EDP_data_next_ptr_=&the_robot->EDP_data.next_XYZ_AA_arm_coordinates[0];
			EDP_data_current_ptr_=&the_robot->EDP_data.current_XYZ_AA_arm_coordinates[0];
//			cout<<"XYZ_ANGLE_AXIS\n";
			atype_=XYZ_ANGLE_AXIS; }
     	if (EDP_data_next_ptr_!=0) {//ntdes_ptr_->arm_type==MOTOR || ntdes_ptr_->arm_type== JOINT || ntdes_ptr_->arm_type==XYZ_EULER_ZYZ || ntdes_ptr_->arm_type== XYZ_ANGLE_AXIS) {
			the_robot->EDP_data.instruction_type = GET;
			the_robot->EDP_data.get_type = ARM_DV;
			the_robot->EDP_data.set_type = ARM_DV;
			the_robot->EDP_data.set_arm_type = atype_;
			the_robot->EDP_data.get_arm_type = atype_;
			the_robot->EDP_data.motion_type = ABSOLUTE;
			 the_robot->EDP_data.next_interpolation_type = MIM;
			the_robot->EDP_data.motion_steps = ntdes_ptr_->internode_step_no;
			the_robot->EDP_data.value_in_step_no = ntdes_ptr_->value_in_step_no; }
		else {
//			cout<<"firststep: dynamic_cast faild \n"<<flush;
			throw ECP_error (NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION); }
//		cout<<"firststep: after dynamic_cast\n"<<flush;
		
  	  

  return true;
}; // end: bool irp6ot_irp6ot_nurbs_generator::first_step ( )

//----------------------------------------------------------------------------------------------
//-----------------------------------  metoda	next_step --------------------------------------
//----------------------------------------------------------------------------------------------

bool irp6ot_nurbs_generator::next_step (  )
{

//	cout<<"nextstep: start \n"<<flush;

   // Kontakt z MP
	if (node_counter >= ntdes_ptr_->interpolation_node_no)  { // Koniec odcinka
     
     	return false;
     	} 



   // Przygotowanie kroku ruchu - do kolejnego wezla interpolacji

   the_robot->EDP_data.instruction_type = SET;
   the_robot->EDP_data.get_type = NOTHING_DV;
   the_robot->EDP_data.get_arm_type = INVALID_END_EFFECTOR;
   the_robot->EDP_data.set_type = ARM_DV; // ARM
   the_robot->EDP_data.set_arm_type = atype_;
   the_robot->EDP_data.motion_type = ABSOLUTE;
    the_robot->EDP_data.next_interpolation_type = MIM;

   
//	cout<<"nextstep: Start2 \n"<<flush;  
//	cout<<"ntdes_ptr_ "<<(int)ntdes_ptr_<<"\n"<<flush;
//	cout<<"ncptr "<<(int)ntdes_ptr_->ncptr<<"\n"<<flush;
	const double max=ntdes_ptr_->ncptr->maxT();		
//	cout<<"nextstep: Start2 maxT() \n"<<flush;  
	const double dT = max * (double)node_counter/ntdes_ptr_->interpolation_node_no;
   	const Point* Pdelta = &(ntdes_ptr_->ncptr->curvePoint( dT ));
	for (unsigned int i=0; i<Dim; i++) {
		EDP_data_next_ptr_[i] = (*Pdelta)[i] + EDP_data_current_ptr_[i]; }
		
	the_robot->EDP_data.next_gripper_coordinate=the_robot->EDP_data.current_gripper_coordinate;	
//	cout<<"nextstep: start3 \n"<<flush;

	// skopiowac przygotowany rozkaz dla EDP do bufora wysylkowego
	
	return true;

}; // end:  irp6ot_nurbs_generator::next_step ( )

} // namespace irp6ot
} // namespace ecp
} // namespace mrrocpp

