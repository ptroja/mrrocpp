/*
 * location.h
 *
 *  Created on: Feb 13, 2012
 *      Author: ptroja
 */

#ifndef ZMQ_LOCATION_H_
#define ZMQ_LOCATION_H_

#include <string>
#include <ostream>

#include <boost/serialization/serialization.hpp>
#include <boost/serialization/string.hpp>

namespace mrrocpp {
namespace lib {
namespace zmq {

class location {
public:
	enum { REGISTER, UNREGISTER, QUERY, PING, ACK, NACK } type;
	std::string name;
	std::string host;
	int port;
	int pid;
	int timer;

	// TODO: Constructors with default values.
private:
	//! Give access to boost::serialization framework
	friend class boost::serialization::access;

	//! Serialization of the data structure
	template <class Archive>
	void serialize(Archive & ar, const unsigned int version)
	{
		ar & type;
		switch(type) {
			case REGISTER:
			case ACK:
			case NACK:
				ar & name;
				ar & host;
				ar & port;
				ar & pid;
				break;
			case QUERY:
			case PING:
			case UNREGISTER:
				ar & name;
				break;
		}
	}

	friend std::ostream& operator<< (std::ostream& stream, const location & me)
	{
		switch(me.type) {
			case REGISTER:
				stream << "REGISTER:" << me.name << "@" << me.host << ":" << me.port << "/" << me.pid;
				break;
			case ACK:
				stream << "ACK:" << me.name;
				break;
			case NACK:
				stream << "NACK:" << me.name;
				break;
			case QUERY:
				stream << "QUERY:" << me.name;
				break;
			case UNREGISTER:
				stream << "UNREGISTER:" << me.name;
				break;
			case PING:
				stream << "PING:" << me.name;
				break;
			default:
				stream << "?:" << me.name;
				break;
		}

		return stream;
	}


};

} // namespace zmq
} // namespace lib
} // namespace mrrocpp

#endif /* ZMQ_LOCATION_H_ */
