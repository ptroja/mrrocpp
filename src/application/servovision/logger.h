/*
 * logger.h
 *
 *  Created on: Apr 21, 2010
 *      Author: mboryn
 */

#ifndef LOGGER_H_
#define LOGGER_H_

namespace logger {

/** Is log enabled*/
extern bool logEnabled, logDbgEnabled;

/**
 * Print message to the console only if logEnabled is set to true.
 * @param fmt printf-like format
 */
void log(const char *fmt, ...);

/**
 * Print message to the console only if logDbgEnabled is set to true.
 * @param fmt printf-like format
 */
void logDbg(const char *fmt, ...);

} // namespace logger

#endif /* LOGGER_H_ */
