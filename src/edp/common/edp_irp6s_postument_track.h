// -------------------------------------------------------------------------
//                                   edp.h
// Definicje struktur danych i metod dla procesu EDP
//
// Ostatnia modyfikacja: 16.04.98
// -------------------------------------------------------------------------

#ifndef __EDP_IRP6S_P_T_H
#define __EDP_IRP6S_P_T_H

#include "edp/common/edp_e_manip.h"


namespace mrrocpp {
namespace edp {
namespace common {

// base class for EDP robots with manipulators postument and on_track


/************************ edp_irp6s_effector ****************************/
class irp6s_postument_track_effector: public manip_effector
{

protected:


    // czy chwytak wlaczony ?
    bool is_gripper_active;

	// numer serwo chwytaka
	unsigned short gripper_servo_nr;

    void compute_frame(const lib::c_buffer &instruction); // obliczenia dla ruchu ramienia (koncowka: FRAME)

    // Przeksztalcenie definicji koncowki z postaci
    // FRAME do postaci XYZ_EULER_ZYZ
    // oraz przepisanie wyniku przeksztalcenia do
    // wewnetrznych struktur danych REPLY_BUFFER
  //  void arm_frame_2_pose_force_torque_at_frame (bool* = NULL);
    // Przeksztalcenie definicji koncowki z postaci
    // FRAME do postaci XYZ_EULER_ZYZ
    // oraz przepisanie wyniku przeksztalcenia do
    // wewnetrznych struktur danych REPLY_BUFFER

public:

	irp6s_postument_track_effector (lib::configurator &_config, lib::robot_name_t l_robot_name );       // konstruktor





    void iterate_macrostep(const lib::JointArray & begining_joints,
       		const lib::Homog_matrix & begining_end_effector_frame,
       		const lib::c_buffer &instruction, const lib::Xyz_Angle_Axis_vector & base_pos_xyz_rot_xyz_vector);


    void create_threads ();

    void set_robot_model (lib::c_buffer &);

    void move_arm (lib::c_buffer &);
    void get_arm_position(bool, lib::c_buffer &);
	void master_order(MT_ORDER nm_task, int nm_tryb);
};

/************************ edp_irp6s_effector ****************************/

} // namespace common
} // namespace edp
} // namespace mrrocpp


#endif
