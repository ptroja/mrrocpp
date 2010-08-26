#if !defined(__MP_T_SK_MR_H)
#define __MP_T_SK_MR_H

/*!
 * @file mp_t_sk_mr.h
 * @brief File contains sk_mr mp_task class declaration of unknown contour following application.
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup edge_following
 */


namespace mrrocpp {
namespace mp {
namespace task {

class sk_mr : public task
{
protected:

public:

	sk_mr(lib::configurator &_config);

	// methods for mp template
	void main_task_algorithm(void);

};

/** @} */// end of edge_following

} // namespace task
} // namespace mp
} // namespace mrrocpp

#endif
