/*
 * receiver.c
 *
 *  Created on: Jun 7, 2010
 *      Author: ptroja
 */

#include <stdio.h>

#include "c_cpn.h"

int
main(int argc, char *argv[])
{
	if(CPN_accept(9000) != 0) {
		fprintf(stderr, "CPN_accept() failed\n");
		return -1;
	}

	printf("Connection from CPN Tools accepted\n");

	for(;;) {
		char * message;

		printf("CPN_receive()...\n");
		message = CPN_receive();

		if(!message) {
			fprintf(stderr, "CPN_receive() failed\n");
			return -1;
		}

		printf("%s\n", message);
	}
	return 0;
}
