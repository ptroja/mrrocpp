/*
 * g_rotate.h
 *
 *  Created on: Apr 13, 2010
 *      Author: mmichnie
 */

#ifndef G_ROTATE_H_
#define G_ROTATE_H_

#include "../../base/ecp/ecp_generator.h"
//#include "lib/mrmath/mrmath.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

/**
 *  generator testowy.
 */

class g_rotate: public common::generator::generator//mrrocpp::ecp::common::generator::generator
{
public:
	g_rotate(common::task::task& _ecp_task);
	virtual ~g_rotate();
	virtual bool first_step();
	virtual bool next_step();

	static const char configSectionName[];

	char GEN_REPLY;

	/**
	 * direction to move:
	 * 0 - -Y up (robot)
	 * 1 -  X right
	 * 2 -  Y down (computer)
	 * 3 - -X left
	 */
	void configure(double new_rot_position);

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
	//double currentGripperCoordinate;
	int index;
	//double r;//promien
	double k;//kat
	double first_trans_vect [3];//polozenie poczatkowe - srodek okregu

	double current_arm_coordinates[lib::MAX_SERVOS_NR];
	double first_arm_coordinates[lib::MAX_SERVOS_NR];
	double rot_position;
	int direction;
	double k_max;
	int sekcja;
};//end class

} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp




#endif /* G_ROTATE_H_ */
