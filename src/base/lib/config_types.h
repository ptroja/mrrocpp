/*!
 * @file configsrv.h
 * @brief Data structures for communication with configuration server
 *
 * @author Piotr Trojanek <piotr.trojanek@gmail.com>
 *
 * @ingroup LIB
 */

#if !defined(_CONFIG_TYPES_H)
#define _CONFIG_TYPES_H

#include <string>
#include <boost/serialization/string.hpp>

#define CONFIGSRV_CHANNEL_NAME			"configsrv"

//! Data structure for passing two property trees
typedef struct _config_query {
	// in request: true if the file change requested, false otherwise
	// in reply: true if success (key found, file changed), false otherwise
	bool flag;
	std::string key;
} config_query_t;

template<class Archive>
void serialize(Archive & ar, config_query_t & query, const unsigned int version)
{
    ar & query.flag;
    ar & query.key;
}

#endif /* _CONFIG_TYPES_H */
