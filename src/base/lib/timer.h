/*!
 * @file timer.h
 * @brief Utility timer class declaration.
 *
 * @author Piotr Trojanek <piotr.trojanek@gmail.com>
 *
 * @ingroup LIB
 */

#ifndef TIMER_H
#define TIMER_H

#include <ctime>

namespace mrrocpp {
namespace lib {

/**
 * 	Utility timer class
 */
class timer
{
public:
	//! Timer status
	typedef enum _timer_status_enum
	{
		TIMER_INITIALIZED,
		TIMER_STARTED,
		TIMER_STOPPED,
		TIME_RETRIVED,
		CYCLES_RETRIVED,
		TIMER_NOT_INITIALIZED,
		TIMER_NOT_STARTED,
		TIMER_NOT_STOPPED,
		TIMER_RUNNING_OR_NOT_STARTED
	} timer_status_t;

private:
	//! Last and current timestamps
	struct timespec t1, t2;

	//! Flag indicating initialization status
	bool initialized;

	//! Flag indicating started status
	bool started;

	//! Flag indicating stopped status
	bool stopped;

	//! Variable for last status
	timer_status_t last_status;

public:
	//! Constructor
	timer();

	//! Start the timer
	timer_status_t start();

	//! Stop the timer
	timer_status_t stop();

	//! Get measured time
	timer_status_t get_time(float & sec);

	//! Print last status
	void print_last_status() const;
};

} // namespace lib
} // namespace mrrocpp

#endif
