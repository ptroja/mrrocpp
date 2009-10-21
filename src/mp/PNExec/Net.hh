/*
 * Net.hh
 *
 *  Created on: Apr 22, 2009
 *      Author: ptroja
 */

#ifndef NET_HH_
#define NET_HH_

#include <map>

#include "Node.hh"
#include "Arc.hh"
#include "Place.hh"
#include "Transition.hh"
#include "PNExecToolSpecific.hh"

#include <boost/thread/thread.hpp>
#include <boost/ptr_container/ptr_map.hpp>

#include <exception>

namespace pnexec {

class DeadlockException : public std::exception {
    private:
        const std::string description;

    public:
    	DeadlockException(std::string _desc) :
            std::exception(), description(_desc)
        {
        }

        virtual const char* what() const throw()
        {
            return description.c_str();
        }

        ~DeadlockException() throw()
        {
        }
};

#define PNML_SCHEMA_XSD	"pipepnml.xsd"

class Net
{
	private:
		typedef boost::ptr_container_detail::ref_pair<NodeId, Place * const> Place_pair_t;
		typedef boost::ptr_container_detail::ref_pair<NodeId, Transition * const> Transition_pair_t;
		typedef boost::ptr_container_detail::ref_pair<NodeId, Arc * const> Arc_pair_t;

		typedef boost::ptr_map<NodeId, Place> places_t;
		typedef boost::ptr_map<NodeId, Transition> transitions_t;
		typedef boost::ptr_map<NodeId, Arc> arcs_t;

		places_t places;
		transitions_t transitions;
		arcs_t arcs;

		// executor thread group
		boost::thread_group executor;

		// task finished condition
		boost::condition_variable cond;
		boost::mutex mtx;

	public:
		Net(void);

		void BuildFromPNML(const char *pnmlfile);
		void ExecuteStep(void);
		void Add(Place * _node);
		void Add(Transition * _node);
		void Add(Arc * _arc);
		~Net();

		void PrintMarkedPlaces(void);
};

} // namespace

#endif /* NET_HH_ */
