/*
 * zmq_names.cc
 *
 *  Created on: Feb 7, 2012
 *      Author: ptroja
 */

#include <string>
#include <iostream>

#include <boost/unordered_map.hpp>

#include <zmq.hpp>

#include "zmqpp.h"

int main(int argc, char *argv[])
{
	// Name to location mapping.
	typedef boost::unordered_map<std::string, zmqpp::location> locations_t;

	locations_t locations;

	// Initialize ZMQ context.
	zmq::context_t ctx(1);

	// Create ZMQ socket.
	zmq::socket_t sock(ctx, ZMQ_REP);

	// Bind to address.
	sock.bind("tcp://*:5555");

	while (true) {
		// Create place-holder data.
		zmqpp::location msg;
#if 0
		// Setup poll list.
		zmq::pollitem_t pollitem;

		pollitem.socket = sock;
		pollitem.events = ZMQ_POLLIN|ZMQ_POLLERR;

		int r = zmq::poll(&pollitem, 1, -1);

		std::cout << r << std::endl;

		if(pollitem.revents & ZMQ_POLLIN) {
			std::cout << "ZMQ_POLLIN" << std::endl;
		}

		if(pollitem.revents & ZMQ_POLLERR) {
			std::cout << "ZMQ_POLLERR" << std::endl;
		}
#endif
		zmqpp::recv(sock, msg);

		std::cout << "<-" << msg << std::endl;

		if(msg.type == zmqpp::location::REGISTER) {

			// Check location within a map.
			if(locations.count(msg.name) == 0) {
				// Update map.
				locations.insert(locations_t::value_type(msg.name, msg));

				// Confirm.
				msg.type = zmqpp::location::ACK;
			} else {
				// Default to NACK message.
				msg.type = zmqpp::location::NACK;
			}

		} else if (msg.type == zmqpp::location::UNREGISTER) {

			// Check location within a map.
			locations_t::const_iterator it = locations.find(msg.name);

			if(it != locations.end()) {
				// Erase.
				locations.erase(it);

				// Confirm.
				msg.type = zmqpp::location::ACK;
			} else {
				// Default to NACK message.
				msg.type = zmqpp::location::NACK;
			}

		} else if (msg.type == zmqpp::location::QUERY) {
			locations_t::const_iterator it = locations.find(msg.name);

			if(it != locations.end()) {
				msg = it->second;

				// Confirm.
				msg.type = zmqpp::location::ACK;
			} else {
				// Default to NACK message.
				msg.type = zmqpp::location::NACK;
			}
		} else {
			// Default to NACK message.
			msg.type = zmqpp::location::NACK;
		}

		std::cout << "->" << msg << std::endl;

		// Reply.
		zmqpp::send(sock, msg);
	}

	return 0;
}
