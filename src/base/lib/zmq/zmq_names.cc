/*
 * zmq_names.cc
 *
 *  Created on: Feb 7, 2012
 *      Author: ptroja
 */

#include <string>
#include <iostream>

#include <boost/unordered_map.hpp>
#include <boost/foreach.hpp>

#include <zmq.hpp>

#include "location.h"
#include "registry.h"

// Name to location mapping.
typedef boost::unordered_map<std::string, mrrocpp::lib::zmq::location> locations_t;

void print_registry(const locations_t & registry)
{
	std::cout << "registry#" << registry.size() << ":" << std::endl;
	BOOST_FOREACH(const locations_t::value_type & item, registry) {
		std::cout << "\t" << item.second << std::endl;
	}
}

void flush_registry(locations_t & registry)
{
	bool changed = false;

	//std::cout << "flush registry" << std::endl;
	for(locations_t::iterator it = registry.begin(); it != registry.end();) {
		if(--(it->second.timer)) {
			it = registry.erase(it);
			changed = true;
		} else {
			++it;
		}
	}

	if(changed) print_registry(registry);
}

int main(int argc, char *argv[])
{
	locations_t locations;

	// Initialize ZMQ context.
	zmq::context_t ctx(1);

	// Create ZMQ socket.
	zmq::socket_t sock(ctx, ZMQ_REP);

	// Bind to address.
	sock.bind("tcp://*:5555");

	while (true) {
		// Display registry.
		print_registry(locations);

		// Create place-holder data.
		mrrocpp::lib::zmq::location msg;

		// Setup poll list.
		zmq::pollitem_t pollitem;

		pollitem.socket = sock;
		pollitem.events = ZMQ_POLLIN;

		int r = zmq::poll(&pollitem, 1, 500000);

		// Timeout.
		if(r == 0) {
			flush_registry(locations);
			continue;
		}

		mrrocpp::lib::zmq::recv(sock, msg);

		//std::cout << "<-" << msg << std::endl;

		if(msg.type == mrrocpp::lib::zmq::location::REGISTER) {

			// Check location within a map.
			if(locations.count(msg.name) == 0) {
				// Update map.
				locations.insert(locations_t::value_type(msg.name, msg));

				// Confirm.
				msg.type = mrrocpp::lib::zmq::location::ACK;
			} else {
				// Default to NACK message.
				msg.type = mrrocpp::lib::zmq::location::NACK;
			}

		} else if (msg.type == mrrocpp::lib::zmq::location::UNREGISTER) {

			// Check location within a map.
			locations_t::const_iterator it = locations.find(msg.name);

			if(it != locations.end()) {
				// Erase.
				locations.erase(it);

				// Confirm.
				msg.type = mrrocpp::lib::zmq::location::ACK;
			} else {
				// Default to NACK message.
				msg.type = mrrocpp::lib::zmq::location::NACK;
			}

		} else if (msg.type == mrrocpp::lib::zmq::location::QUERY) {
			locations_t::const_iterator it = locations.find(msg.name);

			if(it != locations.end()) {
				msg = it->second;

				// Confirm.
				msg.type = mrrocpp::lib::zmq::location::ACK;
			} else {
				// Default to NACK message.
				msg.type = mrrocpp::lib::zmq::location::NACK;
			}
		} else if (msg.type == mrrocpp::lib::zmq::location::PING) {
			locations_t::iterator it = locations.find(msg.name);

			if(it != locations.end()) {
				it->second.timer = 5;

				// Confirm.
				msg.type = mrrocpp::lib::zmq::location::ACK;
			} else {
				// Default to NACK message.
				msg.type = mrrocpp::lib::zmq::location::NACK;
			}
		} else {
			// Default to NACK message.
			msg.type = mrrocpp::lib::zmq::location::NACK;
		}

		//std::cout << "->" << msg << std::endl;

		// Reply.
		mrrocpp::lib::zmq::send(sock, msg);
	}

	return 0;
}
