/*
 * xmltest.cc
 *
 *  Created on: Jun 4, 2011
 *      Author: ptroja
 */

#include "dp_spkm.h"

#include <iomanip>
#include <iostream>
#include <fstream>
#include <string>

#include <cstdio> // remove
#include <boost/config.hpp>
#if defined(BOOST_NO_STDC_NAMESPACE)
namespace std{
    using ::remove;
}
#endif

#include <boost/serialization/vector.hpp>
#include <boost/archive/tmpdir.hpp>

#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/xml_oarchive.hpp>

using namespace mrrocpp::lib::spkm;

typedef std::vector<segment_t> spkm_segment_sequence_t;

void save_sequence(const spkm_segment_sequence_t &spkm_segment_sequence, const char * filename){
    // make an archive
    std::ofstream ofs(filename);
    assert(ofs.good());
    boost::archive::xml_oarchive oa(ofs);
    oa << BOOST_SERIALIZATION_NVP(spkm_segment_sequence);
}

void
restore_sequence(spkm_segment_sequence_t &spkm_segment_sequence, const char * filename)
{
    // open the archive
    std::ifstream ifs(filename);
    assert(ifs.good());
    boost::archive::xml_iarchive ia(ifs);

    // restore the schedule from the archive
    ia >> BOOST_SERIALIZATION_NVP(spkm_segment_sequence);
}

int main(int argc, char *argv[])
{
	spkm_segment_sequence_t spkm_segment_sequence;

	segment_t s1, s2, s3;

	s1.duration = 1.0;
	s2.duration = 2.0;
	s3.duration = 3.0;

	spkm_segment_sequence.push_back(s1);
	spkm_segment_sequence.push_back(s2);
	spkm_segment_sequence.push_back(s3);

	save_sequence(spkm_segment_sequence, "foo.xml");

#if 0
    // make the schedule
    bus_schedule original_schedule;

    // fill in the data
    // make a few stops
    bus_stop *bs0 = new bus_stop_corner(
        gps_position(34, 135, 52.560f),
        gps_position(134, 22, 78.30f),
        "24th Street", "10th Avenue"
    );
    bus_stop *bs1 = new bus_stop_corner(
        gps_position(35, 137, 23.456f),
        gps_position(133, 35, 54.12f),
        "State street", "Cathedral Vista Lane"
    );
    bus_stop *bs2 = new bus_stop_destination(
        gps_position(35, 136, 15.456f),
        gps_position(133, 32, 15.300f),
        "White House"
    );
    bus_stop *bs3 = new bus_stop_destination(
        gps_position(35, 134, 48.789f),
        gps_position(133, 32, 16.230f),
        "Lincoln Memorial"
    );

    // make a  routes
    bus_route route0;
    route0.append(bs0);
    route0.append(bs1);
    route0.append(bs2);

    // add trips to schedule
    original_schedule.append("bob", 6, 24, &route0);
    original_schedule.append("bob", 9, 57, &route0);
    original_schedule.append("alice", 11, 02, &route0);

    // make aother routes
    bus_route route1;
    route1.append(bs3);
    route1.append(bs2);
    route1.append(bs1);

    // add trips to schedule
    original_schedule.append("ted", 7, 17, &route1);
    original_schedule.append("ted", 9, 38, &route1);
    original_schedule.append("alice", 11, 47, &route1);

    // display the complete schedule
    std::cout << "original schedule";
    std::cout << original_schedule;

    std::string filename(boost::archive::tmpdir());
    filename += "/demo.xml";

    // save the schedule
    save_schedule(original_schedule, filename.c_str());

    // ... some time later
    // make  a new schedule
    bus_schedule new_schedule;

    restore_schedule(new_schedule, filename.c_str());

    // and display
    std::cout << "\nrestored schedule";
    std::cout << new_schedule;
    // should be the same as the old one. (except for the pointer values)

    std::remove(filename.c_str());

    delete bs0;
    delete bs1;
    delete bs2;
    delete bs3;
#endif
    return 0;
}
