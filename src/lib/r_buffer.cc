#include "lib/com_buf.h"

namespace mrrocpp {
namespace lib {

r_buffer::r_buffer (void) :
  reply_type(lib::ERROR),
  robot_model_type(TOOL_FRAME),
  arm_type(FRAME),
  input_values(0)
{
  // konstruktor (inicjalizator) bufora odpowiedzi dla ECP

  error_no.error0 = OK;
  error_no.error1 = OK;
}

} // namespace lib
} // namespace mrrocpp
