#if !defined(_ECP_SUBTASK_SPLINE_GEN_TEST_H)
#define _ECP_SUBTASK_SPLINE_GEN_TEST_H

#include "application/generator_tester/ecp_mp_st_spline_gen_test.h"
#include "base/ecp/ecp_generator.h"

namespace mrrocpp {
namespace ecp {
namespace common {

namespace generator {
class spline;

    class spline_gen_test : public common::generator::generator
{

private:
        boost::shared_ptr <spline> spgenjoint;
        boost::shared_ptr <spline> spgenmotor;
        boost::shared_ptr <spline> spgeneuler;
        boost::shared_ptr <spline> spgenangle;

        bool track;
        bool postument;
        bool conv;
        std::string network_path;

public:
        spline_gen_test(task::task & _ecp_t);
        ~spline_gen_test();

        void conditional_execution();
};

} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp


#endif
