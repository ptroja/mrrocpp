/*
 * sender.cc
 *
 *  Created on: Feb 7, 2012
 *      Author: ptroja
 */

#include <iostream>

#include "subscriber.h"

int
main(int argc, char *argv[])
{
	mrrocpp::lib::zmq::subscriber sub("foo");

	int i;

	while(true) {
		sub.receive(i);

		std::cout << i << std::endl;

		//sleep(1);
	}

	return 0;
}


