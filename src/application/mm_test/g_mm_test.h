/*
 * g_mm_test.h
 *
 *  Created on: Apr 13, 2010
 *      Author: mmichnie
 */

#ifndef G_MM_TEST_H_
#define G_MM_TEST_H_

#include "../../base/ecp/ecp_generator.h"
//#include "lib/mrmath/mrmath.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

/**
 *  generator testowy.
 */

class g_mm_test: public common::generator::generator//mrrocpp::ecp::common::generator::generator
{
public:
	g_mm_test(common::task::task& _ecp_task);
	virtual ~g_mm_test();
	virtual bool first_step();
	virtual bool next_step();

	static const char configSectionName[];
protected:
	/** Is log enabled*/
	bool logEnabled;
	/**
	 * Print message to the console only if logEnabled is set to true.
	 * @param fmt printf-like format
	 */
	void log(const char *fmt, ...);
	/**
	 * Check if frame is within constraints.
	 */
private:
	lib::Homog_matrix currentFrame;
	double currentGripperCoordinate;
	int index;
	double r;//promien
	double k;//kat
	double first_trans_vect [3];//polozenie poczatkowe - srodek okregu
};//end class

} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp




#endif /* G_MM_TEST_H_ */
