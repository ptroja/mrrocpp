/*
 * Place.hh
 *
 *  Created on: Apr 27, 2009
 *      Author: ptroja
 */

#ifndef PLACE_HH_
#define PLACE_HH_

#include <boost/thread/thread.hpp>
#include <boost/thread/condition_variable.hpp>

#include "Node.hh"
#include "PlaceTransition.hh"

#include "pipepnml.hxx"

#include <ostream>

namespace pnexec {

class Place: public PlaceTransition
{
	private:
	unsigned int marking;
	unsigned int capacity;

	public:
		Place(NodeId _id,
				std::string _name = std::string(""),
				unsigned int _initial_marking = 0,
				int _capacity = -1);

		Place(const place_t & p);

		unsigned int getMarking() const
		{
			return marking;
		}

		void removeMarker(void) {
			// TODO: check for marking >= 0
			if(marking == 0) {
				throw;
			}
			marking--;
			std::cout << name << "-" << marking << std::endl;
		}

		void addMarker(void) {
			marking++;
			std::cout << name << "+" << marking << std::endl;
		}

		void run(void);

//		virtual ~Place(void) {
//			std::cout << "~Place " << name << std::endl;
//		};

		friend std::ostream& operator<< (std::ostream &out, Place &cPlace);
};

class PlaceExecutor {

	public:
		static void run(Place *_place, boost::condition_variable *_cond, boost::mutex *_mtx);
};

} // namespace

#endif /* PLACE_HH_ */
