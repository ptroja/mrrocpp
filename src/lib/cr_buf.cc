// -------------------------------------------------------------------
//                           cr_buf.cc
// 
//  Konstruktory buforow komunikacyjnych pomiedzy EDP i ECP
// 
// Ostatnia modyfikacja: 16.04.98
// -------------------------------------------------------------------

#include "common/com_buf.h"

/*--------------------------------------------------------------------------*/
c_buffer::c_buffer (void) :
  instruction_type(SYNCHRO),
  set_type(0),
  get_type(0),
  set_rmodel_type(TOOL_FRAME),
  get_rmodel_type(TOOL_FRAME),
  set_arm_type(FRAME),
  get_arm_type(FRAME),
  output_values(0),
  address_byte(0),
  motion_type(ABSOLUTE),
  motion_steps(0),
  value_in_step_no(0)
{
  // konstruktor (inicjalizator) bufora polecen z ECP

  for (int i=0; i<4; i++)
    for (int j=0; j<3; j++)
      if (i==j)
        arm.frame_def.arm_frame_m[j][i] = 1.0;
      else
        arm.frame_def.arm_frame_m[j][i] = 0.0;
        
  for (int i=0; i<4; i++)
    for (int j=0; j<3; j++)
      if (i==j)
        rmodel.tool_frame_def.tool_frame_m[j][i] = 1.0;
      else
        rmodel.tool_frame_def.tool_frame_m[j][i] = 0.0;
 // rmodel.tool_frame_def.address_byte = 0;
}; // end: c_buffer::c_buffer
/*--------------------------------------------------------------------------*/


/*--------------------------------------------------------------------------*/
r_buffer::r_buffer (void) :
  reply_type(ERROR),
  rmodel_type(TOOL_FRAME),
  arm_type(FRAME),
  input_values(0)
{
  // konstruktor (inicjalizator) bufora odpowiedzi dla ECP

  error_no.error0 = OK;
  error_no.error1 = OK;

 // address_byte = 0;
  for (int i=0; i<4; i++)
    for (int j=0; j<3; j++)
      if (i==j)
        arm.frame_def.arm_frame_m[j][i] = 1.0;
      else
        arm.frame_def.arm_frame_m[j][i] = 0.0;
  for (int i=0; i<4; i++)
    for (int j=0; j<3; j++)
      if (i==j)
        rmodel.tool_frame_def.tool_frame_m[j][i] = 1.0;
      else
        rmodel.tool_frame_def.tool_frame_m[j][i] = 0.0;
//  rmodel.tool_frame_def.address_byte = 0;
}; // end: r_buffer::r_buffer
/*--------------------------------------------------------------------------*/
