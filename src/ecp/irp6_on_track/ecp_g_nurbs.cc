// -------------------------------------------------------------------------
//                             ecp.cc
//             Effector Control Process (lib::ECP) - methods
// Funkcje do tworzenia procesow ECP
//
//
// Ostatnia modyfikacja: 24.04.2006
// autor: tstempko
// -------------------------------------------------------------------------

#include <fstream>

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "lib/srlib.h"

//#include "lib/matrix.h"
#include "lib/nurbs.h"
#include "lib/nurbs_tdes.h"
#include "ecp/irp6_on_track/ecp_local.h"
#include "ecp/irp6_on_track/ecp_g_nurbs.h"


namespace mrrocpp {
namespace ecp {
namespace irp6ot {
namespace generator {

const size_t Dim=6;

using namespace NurbsLib;


template< lib::POSE_SPECIFICATION arm_type, size_t D >
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

nurbs::nurbs (common::task::task& _ecp_task,
	 const nurbs_tdes &ntdes, int mp_communication_mode_arg)
: base (_ecp_task) 
{
	mp_communication_mode_=mp_communication_mode_arg;
	ntdes_ptr_ = &ntdes;
};

//----------------------------------------------------------------------------------------------
//---------------------------------    metoda	first_step -------------------------------------
//----------------------------------------------------------------------------------------------

bool nurbs::first_step (  )
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
     	if (EDP_data_next_ptr_!=0) {//ntdes_ptr_->arm_type==MOTOR || ntdes_ptr_->arm_type== lib::JOINT || ntdes_ptr_->arm_type==XYZ_EULER_ZYZ || ntdes_ptr_->arm_type== lib::XYZ_ANGLE_AXIS) {
			the_robot->EDP_data.instruction_type = lib::GET;
			the_robot->EDP_data.get_type = ARM_DV;
			the_robot->EDP_data.set_type = ARM_DV;
			the_robot->EDP_data.set_arm_type = atype_;
			the_robot->EDP_data.get_arm_type = atype_;
			the_robot->EDP_data.motion_type = lib::ABSOLUTE;
			 the_robot->EDP_data.next_interpolation_type = lib::MIM;
			the_robot->EDP_data.motion_steps = ntdes_ptr_->internode_step_no;
			the_robot->EDP_data.value_in_step_no = ntdes_ptr_->value_in_step_no; }
		else {
//			cout<<"firststep: dynamic_cast faild \n"<<flush;
			throw ECP_error (lib::NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION); }
//		cout<<"firststep: after dynamic_cast\n"<<flush;
		
  	  

  return true;
}; // end: bool irp6ot_irp6ot_nurbs_generator::first_step ( )

//----------------------------------------------------------------------------------------------
//-----------------------------------  metoda	next_step --------------------------------------
//----------------------------------------------------------------------------------------------

bool nurbs::next_step (  )
{

//	cout<<"nextstep: start \n"<<flush;

   // Kontakt z MP
	if (node_counter >= ntdes_ptr_->interpolation_node_no)  { // Koniec odcinka
     
     	return false;
     	} 



   // Przygotowanie kroku ruchu - do kolejnego wezla interpolacji

   the_robot->EDP_data.instruction_type = lib::SET;
   the_robot->EDP_data.get_type = NOTHING_DV;
   the_robot->EDP_data.get_arm_type = lib::INVALID_END_EFFECTOR;
   the_robot->EDP_data.set_type = ARM_DV; // ARM
   the_robot->EDP_data.set_arm_type = atype_;
   the_robot->EDP_data.motion_type = lib::ABSOLUTE;
    the_robot->EDP_data.next_interpolation_type = lib::MIM;

   
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

}
} // namespace irp6ot
} // namespace ecp
} // namespace mrrocpp

