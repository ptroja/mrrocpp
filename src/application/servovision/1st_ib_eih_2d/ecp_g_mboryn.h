/*
 * $Id$
 *
 *  Created on: Dec 11, 2009
 *      Author: mboryn
 */

#ifndef ECP_G_MBORYN_H_
#define ECP_G_MBORYN_H_

#include "ecp/common/generator/ecp_generator.h"
#include "lib/mrmath/mrmath.h"

namespace mrrocpp {

namespace ecp {

namespace irp6ot_m {

namespace generator {

/** @addtogroup servovision
 *  @{
 */

class ecp_g_mboryn: public mrrocpp::ecp::common::generator::generator
{
public:
	ecp_g_mboryn(mrrocpp::ecp::common::task::task & _ecp_task);
	virtual ~ecp_g_mboryn();
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
	bool isArmFrameOk(const lib::Homog_matrix& arm_frame);
private:
	lib::sensor *vsp_fradia;

	lib::Homog_matrix currentFrame;
	double currentGripperCoordinate;
	bool currentFrameSaved;

	double Kp;
	double maxT;

	void printInstruction(void);
};

/** @} */// ecp_g_mboryn

} // namespace generator

} // namespace irp6ot

} // namespace ecp

} // namespace mrrocpp

#endif /* ECP_G_MBORYN_H_ */
