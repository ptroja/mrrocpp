/*
 * sender.cc
 *
 *  Created on: Feb 7, 2012
 *      Author: ptroja
 */

#include <iostream>

#include "zmqpp.h"

int
main(int argc, char *argv[])
{
	while(true) {
		{
			zmqpp::publisher r("foo");
			sleep(5);
		}
		sleep(3);
	}

	return 0;
}


