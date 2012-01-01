/*
 * serialization.h
 *
 *  Created on: Dec 31, 2011
 *      Author: ptroja
 */

#ifndef PLAN_SERIALIZATION_H_
#define PLAN_SERIALIZATION_H_

#include <boost/serialization/nvp.hpp>
#include <boost/serialization/base_object.hpp>

#include "plan.hxx"

template<class Archive>
void serialize(Archive & ar, State & item, const boost::serialization::version_type &)
{
    ar & boost::serialization::make_nvp("agent", (int &) item.agent());
    ar & boost::serialization::make_nvp("TBeg", (float &) item.TBeg());
    ar & boost::serialization::make_nvp("TEnd", (float &) item.TEnd());
}

template<class Archive>
void serialize(Archive & ar, Pkm::ItemType & item, const boost::serialization::version_type &)
{
    // invoke serialization of the base class
	ar & boost::serialization::make_nvp("State", boost::serialization::base_object<State>(item));

    ar & boost::serialization::make_nvp("x", item.Xyz_Euler_Zyz()->x());
    ar & boost::serialization::make_nvp("y", item.Xyz_Euler_Zyz()->y());
    ar & boost::serialization::make_nvp("z", item.Xyz_Euler_Zyz()->z());
    ar & boost::serialization::make_nvp("alpha", item.Xyz_Euler_Zyz()->alpha());
    ar & boost::serialization::make_nvp("beta", item.Xyz_Euler_Zyz()->beta());
    ar & boost::serialization::make_nvp("gamma", item.Xyz_Euler_Zyz()->gamma());
}

template<class Archive>
void serialize(Archive & ar, Mbase::ItemType & item, const boost::serialization::version_type &)
{
//    // invoke serialization of the base class
	ar & boost::serialization::make_nvp("State", boost::serialization::base_object<State>(item));

    ar & boost::serialization::make_nvp("numActions", item.numActions());

    Plan::MbaseType::ItemType::ActionsType::ItemIterator it = item.actions().item().begin();

    for(int i = 0; i < item.numActions(); ++i) {
    	ar & boost::serialization::make_nvp("pin", it->pin());
    	ar & boost::serialization::make_nvp("dThetaInd", it->dThetaInd());
    	ar & boost::serialization::make_nvp("dPkmTheta", it->dPkmTheta());
    }
}


#endif /* PLAN_SERIALIZATION_H_ */
