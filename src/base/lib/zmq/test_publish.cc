/*
 * sender.cc
 *
 *  Created on: Feb 7, 2012
 *      Author: ptroja
 */

#include <iostream>

#include "publisher.h"

int
main(int argc, char *argv[])
{
	mrrocpp::lib::zmq::publisher pub("foo");

	while(true) {
		sleep(1);
	}

	return 0;
}


