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
			zmqpp::receiver r("foo");
		}
		sleep(1);
	}

	return 0;
}


