#include "common/com_buf.h"

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
        arm.frame_def.arm_frame[j][i] = 1.0;
      else
        arm.frame_def.arm_frame[j][i] = 0.0;
  for (int i=0; i<4; i++)
    for (int j=0; j<3; j++)
      if (i==j)
        rmodel.tool_frame_def.tool_frame[j][i] = 1.0;
      else
        rmodel.tool_frame_def.tool_frame[j][i] = 0.0;
//  rmodel.tool_frame_def.address_byte = 0;
}
