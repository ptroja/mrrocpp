#if !defined(_SMOOTH_GEN_TEST_H)
#define _SMOOTH_GEN_TEST_H

#include "application/generator_tester/ecp_mp_st_smooth_gen_test.h"
#include "base/ecp/ecp_generator.h"

namespace mrrocpp {
namespace ecp {
namespace common {

namespace generator {
class newsmooth;

    class smooth_gen_test : public common::generator::generator
{

private:
        boost::shared_ptr <newsmooth> sgenjoint;
        boost::shared_ptr <newsmooth> sgenmotor;
        boost::shared_ptr <newsmooth> sgeneuler;
        boost::shared_ptr <newsmooth> sgenangle;

	bool track;
	bool postument;
	bool conv;
	std::string network_path;

public:
        smooth_gen_test(task::task & _ecp_t);
        ~smooth_gen_test();

	void conditional_execution();
};

} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp


#endif
