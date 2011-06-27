/*
 * g_en_labyrinth.h
 *
 * Author: enatil
 */

#ifndef G_EN_LAB_H_
#define G_EN_LAB_H_

#include "../../base/ecp/ecp_generator.h"

#define UP 0
#define RIGHT 1
#define DOWN 2
#define LEFT 3

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {


class g_en_lab: public common::generator::generator//mrrocpp::ecp::common::generator::generator
{
public:
	g_en_lab(common::task::task& _ecp_task);
	virtual ~g_en_lab();
	virtual bool first_step();
	virtual bool next_step();

	static const char configSectionName[];

	char GEN_REPLY;

	void configure(int new_direction, double new_k_max);

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
	double k;
	double first_trans_vect [3];

	int direction;
	double k_max;

};

} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif /* G_EN_LAB_H_ */
