#ifndef EN_LABYRINTH_READING_HPP_
#define EN_LABYRINTH_READING_HPP_

#define MAX_PATH_SIZE 64

#include "application/visual_servoing/Reading.h"

namespace Types {
namespace Mrrocpp_Proxy {


class EN_Labyrinth_Reading: public Reading
{
public:
	EN_Labyrinth_Reading()
	{
//		labyrinth_solved = false;
//		path_size = 0;
//		start_point_x = 0;
//		start_point_y = 0;
//		end_point_x = 0;
//		end_point_y = 0;
//		path[0] = 0;
	}

	EN_Labyrinth_Reading(const EN_Labyrinth_Reading& o)
	{
//		std::cout << "EN_Labyrinth_Reading from MRROCPP copy constructor: " << std::endl;
//		std::cout << "Solved: " << o.labyrinth_solved << std::endl;
//		std::cout << "Path Size: " << o.path_size << std::endl;
//		std::cout << "Start_pt: (" << o.start_point_x << "," << o.start_point_y << ")" << std::endl;
//		std::cout << "End_pt ("  << o.end_point_x << "," << o.end_point_y << ")" << std::endl;

		labyrinth_solved = o.labyrinth_solved;
		path_size = o.path_size;
		start_point_x = o.start_point_x;
		start_point_y = o.start_point_y;
		end_point_x = o.end_point_x;
		end_point_y = o.end_point_y;
		for(int i=0; i<o.path_size; ++i)
			path[i] = o.path[i];
	}

	virtual ~EN_Labyrinth_Reading()
	{
	}

	virtual EN_Labyrinth_Reading* clone()
	{
//		std::cout << "EN_Labyrinth_Reading from MRROCPP clone():" << std::endl;
//		std::cout << "Solved: " << labyrinth_solved << std::endl;
//		std::cout << "Path Size: " << path_size << std::endl;
//		std::cout << "Start_pt: (" << start_point_x << "," << start_point_y << ")" << std::endl;
//		std::cout << "End_pt ("  << end_point_x << "," << end_point_y << ")" << std::endl;

		return new EN_Labyrinth_Reading(*this);
	}

	bool labyrinth_solved;
	int path_size;
	int start_point_x;
	int start_point_y;
	int end_point_x;
	int end_point_y;
	int path[MAX_PATH_SIZE];

	virtual void send(boost::shared_ptr<xdr_oarchive<> > & ar)
	{
		*ar<<*this;
	}

private:

	friend class boost::serialization::access;
	template <class Archive>
	void serialize(Archive & ar, const unsigned int version)
	{
//		std::cout << "EN_Labyrinth_Reading from MRROCPP serialize():" << std::endl;
//		std::cout << "Solved: " << labyrinth_solved << std::endl;
//		std::cout << "Path Size: " << path_size << std::endl;
//		std::cout << "Start_pt: (" << start_point_x << "," << start_point_y << ")" << std::endl;
//		std::cout << "End_pt ("  << end_point_x << "," << end_point_y << ")" << std::endl;

		ar & boost::serialization::base_object <Reading>(*this);
		ar & labyrinth_solved;
		ar & path_size;
		ar & start_point_x;
		ar & start_point_y;
		ar & end_point_x;
		ar & end_point_y;
		ar & path;
	}
};

}//namespace Mrrocpp_Proxy
}//namespace Types

#endif /* EN_LABYRINTH_READING_HPP_ */
