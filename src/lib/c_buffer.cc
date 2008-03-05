#include "common/com_buf.h"

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
				arm.frame_def.arm_frame[j][i] = 1.0;
			else
				arm.frame_def.arm_frame[j][i] = 0.0;

	for (int i=0; i<4; i++)
		for (int j=0; j<3; j++)
			if (i==j)
				rmodel.tool_frame_def.tool_frame[j][i] = 1.0;
			else
				rmodel.tool_frame_def.tool_frame[j][i] = 0.0;
	// rmodel.tool_frame_def.address_byte = 0;
}

// odczytac XXX co?
bool c_buffer::is_get_controller_state() const
{
	return get_type & CONTROLLER_STATE_DV;
}

// odczytac wejscia?
bool c_buffer::is_get_inputs() const
{
	return get_type & OUTPUTS_DV;
}

// odczytac narzedzie?
bool c_buffer::is_get_rmodel() const
{
	return get_type & RMODEL_DV;
}

// odczytac polozenie ramienia?
bool c_buffer::is_get_arm() const
{
	return get_type & ARM_DV;
}

// ustawic wyjscia?
bool c_buffer::is_set_outputs() const
{
	return set_type & OUTPUTS_DV;
}

// zmienic narzedzie?
bool c_buffer::is_set_rmodel() const
{
	return set_type & RMODEL_DV;
}

// zmienic polozenie ramienia?
bool c_buffer::is_set_arm() const
{
	return set_type & ARM_DV;
}