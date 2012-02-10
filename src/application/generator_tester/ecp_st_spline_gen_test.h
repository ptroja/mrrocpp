#if !defined(_ECP_SUBTASK_SPLINE_GEN_TEST_H)
#define _ECP_SUBTASK_SPLINE_GEN_TEST_H

#include "base/ecp/ecp_subtask.h"
#include "application/generator_tester/ecp_mp_st_spline_gen_test.h"

namespace mrrocpp {
namespace ecp {
namespace common {

namespace generator {
class spline;
}

namespace subtask {

class subtask_spline_gen_test : public subtask
{

private:
        generator::spline* spgenjoint;
        generator::spline* spgenmotor;
        generator::spline* spgeneuler;
        generator::spline* spgenangle;

        bool track;
        bool postument;
        bool conv;
        std::string network_path;

public:
        subtask_spline_gen_test(task::task & _ecp_t);
        ~subtask_spline_gen_test();

        void conditional_execution();
};

} // namespace task
} // namespace common
} // namespace ecp
} // namespace mrrocpp


#endif
