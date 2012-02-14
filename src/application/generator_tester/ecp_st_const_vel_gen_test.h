#if !defined(_ECP_SUBTASK_CONST_VEL_GEN_TEST_H)
#define _ECP_SUBTASK_CONST_VEL_GEN_TEST_H

#include "application/generator_tester/ecp_mp_st_const_vel_gen_test.h"
#include "base/ecp/ecp_generator.h"

namespace mrrocpp {
namespace ecp {
namespace common {

namespace generator {
class constant_velocity;

class const_vel_gen_test : public common::generator::generator
{

private:
        boost::shared_ptr <constant_velocity> cvgenjoint;
        boost::shared_ptr <constant_velocity> cvgenmotor;
        boost::shared_ptr <constant_velocity> cvgeneuler;
        boost::shared_ptr <constant_velocity> cvgenangle;

	bool track;
	bool postument;
	bool conv;
        std::string network_path;

public:
        const_vel_gen_test(task::task & _ecp_t);
        ~const_vel_gen_test();

	void conditional_execution();
};

} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp


#endif
