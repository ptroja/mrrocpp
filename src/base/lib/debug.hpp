/*
 * debug.hpp
 *
 *  Created on: 21-12-2011
 *      Author: tkornuta
 */

#ifndef DEBUG_HPP_
#define DEBUG_HPP_

#include <cstdio>

// Debug executed methods.
#ifndef DEBUG_METHODS
	#define DEBUG_METHODS 1
#endif

// Debug retrieved commands.
#ifndef DEBUG_COMMANDS
	#define DEBUG_COMMANDS 1
#endif

// Debug reference frames.
#ifndef DEBUG_FRAMES
	#define DEBUG_FRAMES 1
#endif

// Debug joints.
#ifndef DEBUG_JOINTS
	#define DEBUG_JOINTS 1
#endif

// Debug motors.
#ifndef DEBUG_MOTORS
	#define DEBUG_MOTORS 1
#endif

// Macro for displaying methods.
#if(DEBUG_METHODS)
	#define DEBUG_METHOD \
	fprintf(stderr, "%s [%d] : \033[0;36m%s\033[0;37m\n", __FILE__, __LINE__, __PRETTY_FUNCTION__); \
	fflush(stderr);
#else
	#define DEBUG_METHOD
#endif

// Macro for displaying commands.
#if(DEBUG_COMMANDS)
	#define DEBUG_COMMAND(format, args...) \
	fprintf(stderr, "%s [%d] : \033[0;31m", __FILE__, __LINE__); \
	fprintf (stderr, format , ## args); \
	fprintf(stderr, "\033[0;37m\n"); \
	fflush(stderr);
#else
	#define DEBUG_COMMAND(format, args...)
#endif

// Macro for displaying joints.
#if(DEBUG_JOINTS)
	#define DEBUG_JOINT(format, args...) \
	fprintf(stderr, "%s [%d] : \033[0;31m", __FILE__, __LINE__); \
	fprintf (stderr, format , ## args); \
	fprintf(stderr, "\033[0;37m\n"); \
	fflush(stderr);
#else
	#define DEBUG_JOINT(format, args...)
#endif

#endif /* DEBUG_HPP_ */
