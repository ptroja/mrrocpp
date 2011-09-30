

#ifndef LREADING_HPP_
#define LREADING_HPP_

#include "application/visual_servoing/Reading.h"

#define MAX_LAB_LENGTH 50

namespace Types {
namespace Mrrocpp_Proxy {


/**
 *
 */
class LReading: public Reading
{
public:
	LReading()
	{
	}

	LReading(const LReading& o)
	{
		path_exists = o.path_exists;
		waiting = o.waiting;

		for(int i=0;i<MAX_LAB_LENGTH;i++)
			for(int j=0;j<2;j++)
				path[i][j] = o.path[i][j];
	}

	virtual ~LReading()
	{
	}

	virtual LReading* clone()
	{
		return new LReading(*this);
	}

	bool path_exists;
	int path [MAX_LAB_LENGTH][2];
	bool waiting;

	virtual void send(boost::shared_ptr<xdr_oarchive<> > & ar){
		*ar<<*this;
	}

private:
	friend class boost::serialization::access;
	template <class Archive>
	void serialize(Archive & ar, const unsigned int version)
	{
		ar & boost::serialization::base_object <Reading>(*this);

		ar & path_exists;
		ar & path;
		ar & waiting;
	}
};

}//namespace Mrrocpp_Proxy
}//namespace Types

#endif /* LREADING_HPP_ */
