#if !defined(_ECP_SUB_TASK_SPLINE_GEN_TEST_H)
#define _ECP_SUB_TASK_SPLINE_GEN_TEST_H

#include "base/ecp/ecp_sub_task.h"
#include "application/generator_tester/ecp_mp_st_spline_gen_test.h"

namespace mrrocpp {
namespace ecp {
namespace common {

namespace generator {
class spline;
}

namespace sub_task {

class sub_task_spline_gen_test : public sub_task
{

private:
        generator::spline* cvgenjoint;
        generator::spline* cvgenmotor;
        generator::spline* cvgeneuler;
        generator::spline* cvgenangle;

        bool track;
        bool postument;
        bool poly;
        bool conv;


public:
        sub_task_spline_gen_test(task::task & _ecp_t);
        ~sub_task_spline_gen_test();

        void conditional_execution();
};

} // namespace task
} // namespace common
} // namespace ecp
} // namespace mrrocpp


#endif
