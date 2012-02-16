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
typedef boost::unordered_map <std::string, mrrocpp::lib::zmq::location> locations_t;

void print_registry(const locations_t & registry)
{
	std::cout << "registry#" << registry.size() << ":" << std::endl;
	BOOST_FOREACH(const locations_t::value_type & item, registry)
			{
				std::cout << "\t" << item.second << std::endl;
			}
}

void flush_registry(locations_t & registry)
{
	bool changed = false;

	//std::cout << "flush registry" << std::endl;
	for (locations_t::iterator it = registry.begin(); it != registry.end();) {
		if (--(it->second.timer)) {
			it = registry.erase(it);
			changed = true;
		} else {
			++it;
		}
	}

	if (changed)
		print_registry(registry);
}

int main(int argc, char *argv[])
{
	locations_t locations;

	// Initialize ZMQ context.
	zmq::context_t ctx(1);

	// Create ZMQ socket for synchronous requests.
	zmq::socket_t requests_socket(ctx, ZMQ_REP);

	// And also for and asynchronous keep-alive notifications.
	zmq::socket_t notifications_socket(ctx, ZMQ_SUB);

	// Disable filtering for notification socket.
	notifications_socket.setsockopt(ZMQ_SUBSCRIBE, NULL, 0);

	// Bind synchronous requests socket.
	{
		std::string address = "tcp://*:";
		address += (int) mrrocpp::lib::zmq::registry_port;

		requests_socket.bind(address.c_str());
	}

	// Bind keep-alive socket.
	{
		std::string address = "tcp://*:";
		address += (int) mrrocpp::lib::zmq::keep_alive_port;

		notifications_socket.bind(address.c_str());
	}

	// Handle incoming messages in a loop.
	while (true) {
		// Display registry.
		print_registry(locations);

		// Create place-holder data.
		mrrocpp::lib::zmq::location msg;

		// Setup poll list.
		zmq::pollitem_t pollitems[2];

		pollitems[0].socket = requests_socket;
		pollitems[0].events = ZMQ_POLLIN;

		pollitems[1].socket = notifications_socket;
		pollitems[1].events = ZMQ_POLLIN;

		int r = zmq::poll(pollitems, 2, 500000);

		// Timeout.
		if (r == 0) {
			flush_registry(locations);
			continue;
		}

		// Handle request socket.
		if (pollitems[0].events == ZMQ_POLLIN) {

			mrrocpp::lib::zmq::recv(requests_socket, msg);

			//std::cout << "<-" << msg << std::endl;

			if (msg.type == mrrocpp::lib::zmq::location::REGISTER) {

				// Check location within a map.
				if (locations.count(msg.name) == 0) {
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

				if (it != locations.end()) {
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

				if (it != locations.end()) {
					msg = it->second;

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
			mrrocpp::lib::zmq::send(requests_socket, msg);

		} else if (pollitems[1].events == ZMQ_POLLIN) {

			// Handle keep-alive notification.
			if (msg.type == mrrocpp::lib::zmq::location::PING) {
				locations_t::iterator it = locations.find(msg.name);

				if (it != locations.end()) {
					it->second.timer = 5;
				} else {
					// This should not happen.
					std::cerr << "Keep-alive notification from unregistered name '"
							<< msg.name << "'" << std::endl;
				}
			} else {
				// This should not happen.
				std::cerr << "Unknown message type on the keep-alive notification socket from '"
						<< msg.name << "'" << std::endl;
			}
		}
	}

	// Not reached.

	return 0;
}
