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

#include <boost/array.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ptree_serialization.hpp>
#include <boost/serialization/string.hpp>

#define CONFIGSRV_CHANNEL_NAME			"configsrv"

//! Data structure for passing two property trees
typedef struct _property_trees {
	boost::property_tree::ptree common_file_pt, file_pt;
} property_trees_t;

template<class Archive>
void serialize(Archive & ar, property_trees_t & pt, const unsigned int version)
{
    ar & pt.common_file_pt;
    ar & pt.file_pt;
}

#endif /* _CONFIG_TYPES_H */
