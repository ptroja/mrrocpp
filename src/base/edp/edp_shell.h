/*!
 * \file edp_shell.h
 * \brief File containing the declaration of edp::common::effector class.
 *
 * \author yoyek
 * \date 2011
 *
 */

#ifndef __EDP_SHELL_H
#define __EDP_SHELL_H

#include <boost/shared_ptr.hpp>

#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"

#include "base/edp/edp_typedefs.h"

#include "base/lib/sr/sr_edp.h"
#include "base/lib/configurator.h"

#include "base/lib/exception.h"

namespace mrrocpp {
namespace edp {
namespace common {

/*!
 * \class shell
 * \brief EDP shell class
 *
 * Used in edp master process
 *
 * \author yoyek
 */
class shell
{

private:

	/*!
	 * \brief Reference to configuration object
	 *
	 * It stores data read from ini file.
	 */
	lib::configurator &config;

	/*!
	 * \brief full path to the hardware busy file
	 *
	 */
	std::string hardware_busy_file_fullpath;

	/*!
	 * \brief EDP pid
	 *
	 */
	pid_t my_pid;

public:
	shell(lib::configurator &_config);
	~shell();
	bool detect_hardware_busy(void);

	/*!
	 * \brief Method to close hardware busy notification file
	 *
	 */
	bool close_hardware_busy_file(void);

};

} // namespace common
} // namespace edp
} // namespace mrrocpp

#endif
