/*
 * zmq_names.cc
 *
 *  Created on: Feb 7, 2012
 *      Author: ptroja
 */

#include <string>
#include <iostream>

#include <boost/unordered_map.hpp>
#include <boost/tokenizer.hpp>
#include <boost/lexical_cast.hpp>

#include <zmq.hpp>

int main(int argc, char *argv[])
{
	// Name to location mapping.
	typedef boost::unordered_map<std::string, std::string> locations_t;

	locations_t locations;

	// Initialize ZMQ context.
	zmq::context_t ctx(1);

	// Create ZMQ socket.
	zmq::socket_t sock(ctx, ZMQ_REP);

	// Bind to address.
	sock.bind("tcp://*:5555");

	while (true) {
		// Create message.
		zmq::message_t msg;

		sock.recv(&msg);

		printf("%s\n", (char *) msg.data());

		typedef boost::char_separator<char> separator_t ;
		typedef boost::tokenizer<separator_t> tokenizer_t;

		std::string request((char *) msg.data());

		separator_t sep(":/@");

		tokenizer_t tokens(request, sep);

		tokenizer_t::const_iterator it = tokens.begin();

		const std::string type = *it++;

		if(type == "R") {
			const std::string name = *it++;
			const std::string host = *it++;
			const int port = boost::lexical_cast<int>(*it++);
			const int pid = boost::lexical_cast<int>(*it++);

			std::cout << type << ":" << name << "@" << host << ":" << port << "/" << pid << std::endl;

			if(locations.count(name) > 0) {
				// Reply with empty message.
				zmq::message_t reply;

				sock.send(reply);
			} else {
				std::stringstream ss;

				ss << host << ":" << port << "/" << pid;

				locations.insert(locations_t::value_type(name, ss.str()));

				zmq::message_t reply(ss.str().size());

				strcpy((char *) reply.data(), ss.str().c_str());

				sock.send(msg);
			}

		} else if (type == "U") {
			const std::string name = *it++;
			const std::string host = *it++;
			const int port = boost::lexical_cast<int>(*it++);
			const int pid = boost::lexical_cast<int>(*it++);

			std::cout << type << ":" << name << ":" << host << ":" << port << "/" << pid << std::endl;

			locations_t::const_iterator it = locations.find(name);

			if(it == locations.end()) {
				// Reply with empty message.
				zmq::message_t reply;

				sock.send(reply);
			} else {
				zmq::message_t reply(it->second.size());

				strcpy((char *) reply.data(), it->second.c_str());

				sock.send(msg);

				locations.erase(it);
			}

		} else if (type == "Q") {
			const std::string name = *it++;

			locations_t::const_iterator it = locations.find(name);

			if(it == locations.end()) {
				// Reply with empty message.
				zmq::message_t reply;

				sock.send(reply);
			} else {
				zmq::message_t reply(it->second.size());

				strcpy((char *) reply.data(), it->second.c_str());

				sock.send(msg);
			}

		} else {
			// Reply with empty message.
			zmq::message_t reply;

			sock.send(reply);
		}
	}

	return 0;
}
