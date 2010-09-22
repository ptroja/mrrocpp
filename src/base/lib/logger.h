/**
 * \file logger.h
 * \brief Logging utilities.
 * \bug Not multi-thread safe
 *
 * \author Mateusz Boryń <mateusz.boryn@gmail.com>
 */

#ifndef LOGGER_H_
#define LOGGER_H_

namespace logger {

/** Is log enabled*/
extern bool log_enabled, log_dbg_enabled;

/**
 * Print message to the console only if logEnabled is set to true.
 * @param fmt printf-like format
 */
void log(const char *fmt, ...);

/**
 * Print message to the console only if logDbgEnabled is set to true.
 * @param fmt printf-like format
 */
void log_dbg(const char *fmt, ...);

} // namespace logger

#endif /* LOGGER_H_ */
